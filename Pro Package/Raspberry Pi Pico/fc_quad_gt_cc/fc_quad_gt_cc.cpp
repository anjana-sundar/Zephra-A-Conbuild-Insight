#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include <hardware/uart.h>
#include <hardware/dma.h>
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"


#define Main_Loop_Time_MICRO_SECONDS 5000

//--------------EKF--------------//
#define mag_inclination 33.9363060f

#define MPU6050_LSB_PER_g 16384.0f
#define accel_offset_x  0.016940f
#define accel_offset_y -0.015371f
#define accel_offset_z -0.103156f
const float accel_cal[3][3] = {
    {  1.000822f, -0.000070f, -0.000529f },
    { -0.000070f,  0.999569f, -0.001073f },
    { -0.000529f, -0.001073f,  1.003285f }
};



//--------------CompassCal--------------//
#define QMC5883_LSB_PER_G 12000.0f
#define QMC5883_Gauss_to_uT 100.0f

float compass_offset_x = 3.522593f;
float compass_offset_y = -7.581154f;
float compass_offset_z = 11.464800f;
float mag_cal[3][3] = {
    {  0.997198f, -0.034667f, -0.000293f },
    { -0.034667f,  1.034516f, -0.016239f },
    { -0.000293f, -0.016239f,  1.001262f }
};

#define compass_Cal_buff_max_expected_len 100
uint8_t compassCalHoldBuff[compass_Cal_buff_max_expected_len];
bool isCompassCalibrated = false;
float cal_arr[9];
//---------------------------------------//

static int addr = 0x68;
static int addr1 = 0x0D;
float loop_time;
uint32_t time, timePrev;
int16_t accel[3], gyro[3], mag[3];
int32_t gyro_cal[3];
float sigma, ax,ay,az,mx,my,mz, gx,gy,gz,ry,rz;
float wx,wy,wz, qw,qx,qy,qz;
float P[4][4];
float R[2];
float roll,pitch,yaw, wx_crct, wy_crct, wz_crct, q_prev[4];
//leveling :
float   roll_offset_angle, pitch_offset_angle,   q_leveled[4], q_level_rot[4] = { 1, 0, 0, 0 };
//-------------------------------//


//--------------PID--------------//
#define Roll_PID_lim 400
#define Pitch_PID_lim 400
#define Yaw_PID_lim 400
#define angle_to_rate_gain 3.0f
int Roll_PID, Pitch_PID, Yaw_PID;
float pid_Integral[3], pid_Derivative[3];
float error[3], prev_error[3], setpoint_free_error[3], setpoint_free_prev_error[3], desired_angular_rate[3];
float twoX_P_gain = 1.7f, twoX_I_gain = 0.0045f, twoX_D_gain = 0.03f;
float Yaw_P_gain = 1.2f, Yaw_I_gain = 0.0045f, Yaw_D_gain = 0.03f;
float ctrl_roll = 0, ctrl_pitch = 0, ctrl_yaw = 0;
//-------------------------------//

//--------------PWM--------------//
int motor_out[4] = { 0, 0, 0, 0};
int ctrl_channel[4] = { 50, 50, 0, 50};
//-------------------------------//


//-------------UART0--------------//
#define uart0_in_buff_size	13
#define uart0_out_buff_size 34
uint8_t uart0_in_buff[uart0_in_buff_size], uart0_out_buff[uart0_out_buff_size];
bool uart0_is_receiving = false;
volatile uint32_t uart0_irq_time_stamp;
#define not_recvd_since_threshold   1000000 /* in microseconds : t*1000*1000 : t is in seconds */
//-------------------------------//


//--------------GPS--------------//
#define gps_buf_max_expctd_len 500
uint8_t gps_raw_buff[gps_buf_max_expctd_len];
int gps_array_pos = 0; // exclusive acccess by dma0
int dma_chan1 = 0;
uint8_t Lattitude[10], Longitude[11], Sat_count, Fix_type, HDOP[3], GND_velocity[5];
uint8_t gps_decode_loop_shape_count;

uint32_t time_Stamp, Prev_time_Stamp;
//-------------------------------//


//-------GPS_Position_Hold-------//
#define Lattitude_shifted_myGPS_offset  -0.0000345f
#define Longitude_shifted_myGPS_offset  -0.0000717f
#define ctrl_stick_to_velocity_div_gain  0.1f

#define GPHC_MAX_Horizontal_Velocity        8.0f        // in meters per second.
#define GPHC_Vehicle_MAX_Lean_Angle         25.0f        // in degrees

bool is_GPS_mode_ON = false, is_GPS_Hold_once_run_done = false;
float GPHC_P_gain = 7.0f, GPHC_I_gain = 0.03f, GPHC_D_gain = 7.0f;
double LAT_DEG_TO_METERS=0, LON_DEG_TO_METERS=0;
double Vehicle_Lattitude=0, Vehicle_Longitude=0, Vehicle_Lattitude_Prev=0, Vehicle_Longitude_Prev=0, Vehicle_desired_Lattitude=0, Vehicle_desired_Longitude=0;
float velocity_vector_north_frame[2], velocity_vector_north_frame_Prev[2];
float v_p_n_f_error[2];
float GPHC_roll_P, GPHC_roll_I, GPHC_roll_D,  GPHC_pitch_P, GPHC_pitch_I, GPHC_pitch_D,  GPHC_roll_angle_out_body, GPHC_pitch_angle_out_body,  GPHC_roll_angle_out_north, GPHC_pitch_angle_out_north;
float Vehicle_desired_lattitude_body_frame_increment, Vehicle_desired_longitude_body_frame_increment,Vehicle_desired_lattitude_north_frame_increment, Vehicle_desired_longitude_north_frame_increment, yaw_rad;
//-------------------------------//

//--------------LED--------------//
uint8_t loop_div_counter;
uint8_t counter_temp = 0;
//-------------------------------//

//--------------CMD--------------//
uint8_t in_cmd[2];
uint8_t out_status[8];
//-------------------------------//


struct Quaternion{
    float w, x, y, z;
};

struct Angles{
    float roll, pitch, yaw;
};

Quaternion quat_inv(Quaternion q){
    static Quaternion q_inv;
    q_inv.w = q.w;
    q_inv.x = -q.x;
    q_inv.y = -q.y;
    q_inv.z = -q.z;
    return q_inv;
}

Quaternion q1_dot_q2 ( float a, float b, float c, float d,     float e, float f, float g, float h ){
    static Quaternion q;
    q.w = (a*e - b*f - c*g - d*h);
    q.x = (a*f + b*e + c*h - d*g);
    q.y = (a*g - b*h + c*e + d*f);
    q.z = (a*h + b*g - c*f + d*e);
    return q;
}

Quaternion get_Quaternion_from_bodyframe_angles( float roll, float pitch, float yaw ){
    Quaternion q_ = q1_dot_q2( cosf(yaw*0.0174532925f*0.5f), 0, 0, sinf(yaw*0.0174532925f*0.5f),     cosf(roll*0.0174532925f*0.5f), 0, sinf(roll*0.0174532925f*0.5f), 0 );
    Quaternion q = q1_dot_q2( q_.w, q_.x, q_.y, q_.z,     cosf(pitch*0.0174532925f*0.5f), sinf(pitch*0.0174532925f*0.5f), 0, 0 );
    return q;
}

Angles get_error_angles_from_Quaternion(Quaternion q){
    static Angles angles;
    // angles.roll = atan2f( 2.0f * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y) ) * 57.2957795f;
    // angles.pitch = (2.0f * atan2f(sqrt(1.0f + 2.0f * (q.w * q.y - q.x * q.z)), sqrt(1.0f - 2.0f * (q.w * q.y - q.x * q.z))) - M_PI/2.0f) * 57.2957795f;
    // angles.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 57.2957795f;

    angles.pitch = 57.2958f*atan2(2*(q.w*q.x + q.y*q.z) , 1 - 2*(q.x*q.x + q.y*q.y));
    angles.roll  = 57.2958f*asin(2*(q.w*q.y - q.z*q.x));
    angles.yaw   = 57.2958f*atan2( (2*(q.w*q.z + q.x*q.y)) , (1 - 2*(q.y*q.y + q.z*q.z)));

    // float temp_x, temp_y;

    return angles;
}

static void Led_init(){
    sleep_ms(1);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static void Led_set(){
    loop_div_counter++;
    if(uart0_is_receiving){
        if(is_GPS_mode_ON){
            if(loop_div_counter == 30){ gpio_put(PICO_DEFAULT_LED_PIN,1); }
            else if(loop_div_counter == 10 || loop_div_counter == 40){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
            else if(loop_div_counter == 140){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
        }
        else if(!is_GPS_mode_ON){
            if(loop_div_counter == 10 ){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
            else if(loop_div_counter == 140){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
        }

    }
    else if(!uart0_is_receiving){
            if(loop_div_counter == 125){ gpio_put(PICO_DEFAULT_LED_PIN,0); }
        else if(loop_div_counter == 250){ gpio_put(PICO_DEFAULT_LED_PIN,1); loop_div_counter = 0; }
    }
    
}

static void Is_uart0_receiving_and_Action(){
    if ( ( time_us_32() - uart0_irq_time_stamp ) <= not_recvd_since_threshold ) { uart0_is_receiving = true; }
    else if ( ( time_us_32() - uart0_irq_time_stamp ) > not_recvd_since_threshold ) { uart0_is_receiving = false; }

    if(!uart0_is_receiving){ ctrl_channel[0] =50; ctrl_channel[1] =50; ctrl_channel[2] =0; ctrl_channel[3] =50; }
}

static void UART0_irq_OnRecv(){
    if(isCompassCalibrated){
        if(uart_is_readable(uart0)){
            uart_read_blocking(uart0, uart0_in_buff, uart0_in_buff_size);
            if(uart_is_writable(uart0)){
                uart_write_blocking(uart0, uart0_out_buff, uart0_out_buff_size);
            }
            uart0_irq_time_stamp = time_us_32();
        }
    }
    else {
        if(uart_is_readable(uart0)){
            uart_read_blocking(uart0, compassCalHoldBuff, compass_Cal_buff_max_expected_len);   isCompassCalibrated = true;
            if(uart_is_writable(uart0)){
                uart_write_blocking(uart0, compassCalHoldBuff, compass_Cal_buff_max_expected_len);
            }
            uart0_irq_time_stamp = time_us_32();
        }
    }
}

static void UART0_setup(int baud){
    sleep_ms(20);
    uart_init(uart0, 2400);
    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(uart0, baud);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, false);
    irq_set_exclusive_handler(UART0_IRQ, UART0_irq_OnRecv);
    irq_set_priority(UART0_IRQ, 1);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);
    sleep_ms(20);
}

static void GPS_decode(){
    gps_decode_loop_shape_count++;
    if(gps_decode_loop_shape_count == 20){      // @10Hz loop
        for (uint16_t i = 0; i < gps_buf_max_expctd_len; i++){
            if (gps_raw_buff[i] == '$' && (gps_buf_max_expctd_len - i) >= 10){
                //gga
                if (gps_raw_buff[i+2] == 'N' && gps_raw_buff[i+4] == 'G' && gps_raw_buff[i+16] == ',' && gps_raw_buff[i+27] == ',' && gps_raw_buff[i+41] == ',' && gps_raw_buff[i+45] == ',' && gps_raw_buff[i+48] == ',' && gps_raw_buff[i+53] == ','){
                    for (uint8_t k = 0; k < 10; k++){
                        Lattitude[k] = gps_raw_buff[i + 17 + k];
                        Longitude[k] = gps_raw_buff[i + 30 + k];
                    }
                    Longitude[10] = gps_raw_buff[i + 40]; //due to 10lat and 11lon
                    Sat_count = 10*(gps_raw_buff[i + 46] - (uint8_t)48) + (gps_raw_buff[i + 47] - (uint8_t)48);
                    HDOP[0] = gps_raw_buff[i + 49]; HDOP[1] = gps_raw_buff[i + 50];
                    HDOP[2] = gps_raw_buff[i + 51];
                }
                //gsa
                if (gps_raw_buff[i+2] == 'N' && gps_raw_buff[i+4] == 'S' && gps_raw_buff[i+6] == ',' && gps_raw_buff[i+8] == ',' && gps_raw_buff[i+10] == ','){
                    Fix_type = gps_raw_buff[i+9];
                }
            }
        }
        gps_decode_loop_shape_count = 0;

        if(Fix_type != '1'){
            //-------latlon char to float-------//
            double temp = 0;
            temp += (double) (10 * (Lattitude[2] - 48));
            temp += (double) (Lattitude[3] - 48);
            temp += (double) ((double) (Lattitude[5] - 48) / (double) 10);
            temp += (double) ((double) (Lattitude[6] - 48) / (double) 100);
            temp += (double) ((double) (Lattitude[7] - 48) / (double) 1000);
            temp += (double) ((double) (Lattitude[8] - 48) / (double) 10000);
            temp += (double) ((double) (Lattitude[9] - 48) / (double) 100000);
            temp = temp / (double) 60;
            Vehicle_Lattitude = (double) (10 * (Lattitude[0] - 48)) + (double) (Lattitude[1] - 48) + temp;
            temp = 0;

            temp += (double) (10 * (Longitude[3] - 48));
            temp += (double) (Longitude[4] - 48);
            temp += (double) ((double) (Longitude[6] - 48) / (double) 10);
            temp += (double) ((double) (Longitude[7] - 48) / (double) 100);
            temp += (double) ((double) (Longitude[8] - 48) / (double) 1000);
            temp += (double) ((double) (Longitude[9] - 48) / (double) 10000);
            temp += (double) ((double) (Longitude[10] - 48) / (double) 100000);
            temp = temp / (double) 60;
            Vehicle_Longitude = (double) (100 * (Longitude[0] - 48)) + (double) (10 * (Longitude[1] - 48)) + (double) (Longitude[2] - 48) + temp;
            temp = 0;
            //----------------------------------//
            Vehicle_Lattitude = Vehicle_Lattitude - Lattitude_shifted_myGPS_offset;
            Vehicle_Longitude = Vehicle_Longitude - Longitude_shifted_myGPS_offset;

        }else{ Vehicle_Lattitude = 0; Vehicle_Longitude = 0; }
    }
}

static void UART1_setup(int baud){
    sleep_ms(50);
    uart_init(uart1, 2400);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    int __unused actual = uart_set_baudrate(uart1, baud);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, false);
    sleep_ms(20);
}

static void GPS_init(){

    uint8_t set_gps_uart_baud[28]= { 0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x10,
        0x0E, 0x00, 0x23, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0x3A };
    while(!uart_is_writable(uart1));
    if(uart_is_writable(uart1)){
        uart_write_blocking(uart1, set_gps_uart_baud, 28);   // 921600 baudrate
    }
    sleep_ms(50);
    uart_set_baudrate(uart1, 921600);
    sleep_ms(50);

    uint8_t set_gps_update_freq_10Hz[14] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };
    while(!uart_is_writable(uart1));
    if(uart_is_writable(uart1)){
        uart_write_blocking(uart1, set_gps_update_freq_10Hz, 14); // 10 Hz
    }
    sleep_ms(5);

    uint8_t gps_disable_gsv[16] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
    while(!uart_is_writable(uart1));
    if(uart_is_writable(uart1)){
        uart_write_blocking(uart1, gps_disable_gsv, 16);
    }
    sleep_ms(5);

    uint8_t gps_disable_vtg[16] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 };
    while(!uart_is_writable(uart1));
    if(uart_is_writable(uart1)){
        uart_write_blocking(uart1, gps_disable_vtg, 16);
    }
    sleep_ms(5);
    
}

static void DMA0_irq_handler() {
    dma_hw->ints0 = 1u << dma_chan1;
    dma_channel_set_write_addr(dma_chan1, &gps_raw_buff[gps_array_pos], true);
    gps_array_pos = (gps_array_pos+1)%gps_buf_max_expctd_len;
}

static void DMA0_configure() {
    int dma_chan1 = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_chan1);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, uart_get_dreq(uart1, false));
    dma_channel_configure(
        dma_chan1,
        &config,
        NULL,
        &uart1_hw->dr,
        1,
        false
    );
    dma_channel_set_irq0_enabled(dma_chan1, true);
    irq_set_exclusive_handler(DMA_IRQ_0, DMA0_irq_handler);
    irq_set_priority(UART0_IRQ, 0);
    irq_set_enabled(DMA_IRQ_0, true);
    DMA0_irq_handler();
    sleep_ms(50);
}

static void PWM_out_init(){
    gpio_set_function(6, GPIO_FUNC_PWM);
    gpio_set_function(7, GPIO_FUNC_PWM);
    gpio_set_function(8, GPIO_FUNC_PWM);
    gpio_set_function(9, GPIO_FUNC_PWM);
    pwm_set_clkdiv(3,10);
    pwm_set_wrap(3,62500);
    pwm_set_enabled(3, true);
    pwm_set_clkdiv(4,10);
    pwm_set_wrap(4,62500);
    pwm_set_enabled(4, true);
}

static void PWM_Write(){
    pwm_set_chan_level(3, PWM_CHAN_A, int(12500.0f+12.5f*motor_out[0]));
    pwm_set_chan_level(3, PWM_CHAN_B, int(12500.0f+12.5f*motor_out[1]));
    pwm_set_chan_level(4, PWM_CHAN_A, int(12500.0f+12.5f*motor_out[2]));
    pwm_set_chan_level(4, PWM_CHAN_B, int(12500.0f+12.5f*motor_out[3])); 
}

static void I2C_Init(){
    sleep_ms(1);
    i2c_init(i2c0, 400000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
}

static void mpu6050_init() {
    uint8_t power_reg[] = {0x6B, 0x00};
    uint8_t accel_scale[] = {0x1C, 0x00};
    uint8_t gyro_scale[] = {0x1B, 0x18};
    uint8_t low_pass[] = {0x1A, 0x03};
    i2c_write_blocking(i2c0, addr, power_reg, 2, false);
    i2c_write_blocking(i2c0, addr, accel_scale, 2, false);
    i2c_write_blocking(i2c0, addr, gyro_scale, 2, false);
    i2c_write_blocking(i2c0, addr, low_pass, 2, false);
}

static void mpu6050_read_raw() {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c0, addr, &val, 1, true);
    i2c_read_blocking(i2c0, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++) { accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]); }
    val = 0x43;
    i2c_write_blocking(i2c0, addr, &val, 1, true);
    i2c_read_blocking(i2c0, addr, buffer, 6, false);  
    for (int i = 0; i < 3; i++) { gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]); }
}

static void qmc5883_init(){
    uint8_t qmc_reg1[] = {0x0B, 0x01};
    uint8_t qmc_reg2[] = {0x09, 0x0D};
    i2c_write_blocking(i2c0, addr1, qmc_reg1, 2, false);
    i2c_write_blocking(i2c0, addr1, qmc_reg2, 2, false);
}

static void qmc5883_read_raw(){
   uint8_t buffer[6];
    uint8_t val = 0x00;
    i2c_write_blocking(i2c0, addr1, &val, 1, true);
    i2c_read_blocking(i2c0, addr1, buffer, 6, false);
    for (int i = 0; i < 3; i++) { mag[i] = (buffer[(i * 2) + 1] << 8 | buffer[i * 2]); }
}

static void IMU_Read(){
    mpu6050_read_raw();
    qmc5883_read_raw();

    ax = (float)accel[0] / MPU6050_LSB_PER_g;
    ay = (float)accel[1] / MPU6050_LSB_PER_g;
    az = (float)accel[2] / MPU6050_LSB_PER_g;
    ax = ax - accel_offset_x;
    ay = ay - accel_offset_y;
    az = az - accel_offset_z;
    float temp_x = ax * accel_cal[0][0] + ay * accel_cal[0][1] + az * accel_cal[0][2];
    float temp_y = ax * accel_cal[1][0] + ay * accel_cal[1][1] + az * accel_cal[1][2];
    float temp_z = ax * accel_cal[2][0] + ay * accel_cal[2][1] + az * accel_cal[2][2];
    ax = temp_x; ay = temp_y; az = temp_z;
    float norm = sqrtf( ax*ax + ay*ay + az*az );
    ax = ax/norm; ay = ay/norm; az = az/norm;

    mx = ( (float)mag[0] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    my = ( (float)mag[1] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    mz = ( (float)mag[2] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
    mx = mx - compass_offset_x;
    my = my - compass_offset_y;
    mz = mz - compass_offset_z;
    temp_x = mx * mag_cal[0][0] + my * mag_cal[0][1] + mz * mag_cal[0][2];
    temp_y = mx * mag_cal[1][0] + my * mag_cal[1][1] + mz * mag_cal[1][2];
    temp_z = mx * mag_cal[2][0] + my * mag_cal[2][1] + mz * mag_cal[2][2];
    mx = temp_x; my = temp_y; mz = temp_z;
    norm = sqrtf( mx*mx + my*my + mz*mz );
    mx = mx/norm; my = my/norm; mz = mz/norm;

    wx = (gyro[0]-gyro_cal[0])*0.00106585038f;
    wy = (gyro[1]-gyro_cal[1])*0.00106585038f;
    wz = (gyro[2]-gyro_cal[2])*0.00106585038f;
}

static void EKF_Init(){
    sigma = 0.09f;
    R[0] = 0.25f;
    R[1] = 0.64f;
    P[0][0] = 1.0f;
    P[0][1] = 0;
    P[0][2] = 0;
    P[0][3] = 0;
    P[1][0] = 0;
    P[1][1] = 1.0f;
    P[1][2] = 0;
    P[1][3] = 0;
    P[2][0] = 0;
    P[2][1] = 0;
    P[2][2] = 1.0f;
    P[2][3] = 0;
    P[3][0] = 0;
    P[3][1] = 0;
    P[3][2] = 0;
    P[3][3] = 1.0f;  
    gx = 0;  gy = 0;  gz = 1; //ENU 
    ry = cosf(0.0174532925f*mag_inclination);  rz = -sinf(0.0174532925f*mag_inclination); //ENU

    //  for(int i=0; i<2000; i++){
    //      mpu6050_read_raw();  
    //      gyro_cal[1] += gyro[1];
    //      gyro_cal[0] += gyro[0];
    //      gyro_cal[2] += gyro[2];
    //      sleep_ms(1);
    // }
    // gyro_cal[1] = gyro_cal[1]/2000;
    // gyro_cal[0] = gyro_cal[0]/2000;
    // gyro_cal[2] = gyro_cal[2]/2000;

    qw=1; qx=0; qy=0; qz=0;
    sleep_ms(500);
}

static void EKF_Run(){
//--------------------------------------- Predict --------------------------------------//
    float FP[4][4];
    float FPFT[4][4];
    float W[4][3];
    float F[4][4];
    float WWT[4][4];
    float Q[4][4];
    float v[6];
    float h[6];
    float H[6][4];
    float S_inv[6][6];
    float PHT[4][6];
    float K[4][6];
    float I[4][4];
    float I_KHP[4][4];
    float Q_out[4];

    W[0][0] = -qx;
    W[0][1] = -qy;
    W[0][2] = -qz;
    W[1][0] = qw;
    W[1][1] = -qz;
    W[1][2] = qy;
    W[2][0] = qz;
    W[2][1] = qw;
    W[2][2] = -qx;
    W[3][0] = -qy;
    W[3][1] = qx;
    W[3][2] = qw;

    float qw_cap = qw + (loop_time*0.5f)*(- wx*qx - wy*qy - wz*qz);
    float qx_cap = qx + (loop_time*0.5f)*(  wx*qw - wy*qz + wz*qy);
    float qy_cap = qy + (loop_time*0.5f)*(  wx*qz + wy*qw - wz*qx);
    float qz_cap = qz + (loop_time*0.5f)*(- wx*qy + wy*qx + wz*qw);
    float norm_q = sqrt(qw_cap*qw_cap + qx_cap*qx_cap + qy_cap*qy_cap + qz_cap*qz_cap);
    qw = qw_cap/norm_q;
    qx = qx_cap/norm_q;
    qy = qy_cap/norm_q;
    qz = qz_cap/norm_q;
    
    float half_loop = loop_time*0.5f;
    F[0][0] = 1;
    F[0][1] = -wx*half_loop;
    F[0][2] = -wy*half_loop;
    F[0][3] = -wz*half_loop;
    F[1][0] = wx*half_loop;
    F[1][1] = 1;
    F[1][2] = wz*half_loop;
    F[1][3] = -wy*half_loop;
    F[2][0] = wy*half_loop;
    F[2][1] = -wz*half_loop;
    F[2][2] = 1;
    F[2][3] = wx*half_loop;
    F[3][0] = wz*half_loop;
    F[3][1] = wy*half_loop;
    F[3][2] = -wx*half_loop;
    F[3][3] = 1;

    FP[0][0] = (P[0][0] + F[0][1]*P[1][0] + F[0][2]*P[2][0] + F[0][3]*P[3][0]);
    FP[0][1] = (P[0][1] + F[0][1]*P[1][1] + F[0][2]*P[2][1] + F[0][3]*P[3][1]);
    FP[0][2] = (P[0][2] + F[0][1]*P[1][2] + F[0][2]*P[2][2] + F[0][3]*P[3][2]);
    FP[0][3] = (P[0][3] + F[0][1]*P[1][3] + F[0][2]*P[2][3] + F[0][3]*P[3][3]);
    FP[1][0] = (F[1][0]*P[0][0] + P[1][0] + F[1][2]*P[2][0] + F[1][3]*P[3][0]);
    FP[1][1] = (F[1][0]*P[0][1] + P[1][1] + F[1][2]*P[2][1] + F[1][3]*P[3][1]);
    FP[1][2] = (F[1][0]*P[0][2] + P[1][2] + F[1][2]*P[2][2] + F[1][3]*P[3][2]);
    FP[1][3] = (F[1][0]*P[0][3] + P[1][3] + F[1][2]*P[2][3] + F[1][3]*P[3][3]);
    FP[2][0] = (F[2][0]*P[0][0] + F[2][1]*P[1][0] + P[2][0] + F[2][3]*P[3][0]);
    FP[2][1] = (F[2][0]*P[0][1] + F[2][1]*P[1][1] + P[2][1] + F[2][3]*P[3][1]);
    FP[2][2] = (F[2][0]*P[0][2] + F[2][1]*P[1][2] + P[2][2] + F[2][3]*P[3][2]);
    FP[2][3] = (F[2][0]*P[0][3] + F[2][1]*P[1][3] + P[2][3] + F[2][3]*P[3][3]);
    FP[3][0] = (F[3][0]*P[0][0] + F[3][1]*P[1][0] + F[3][2]*P[2][0] + P[3][0]);
    FP[3][1] = (F[3][0]*P[0][1] + F[3][1]*P[1][1] + F[3][2]*P[2][1] + P[3][1]);
    FP[3][2] = (F[3][0]*P[0][2] + F[3][1]*P[1][2] + F[3][2]*P[2][2] + P[3][2]);
    FP[3][3] = (F[3][0]*P[0][3] + F[3][1]*P[1][3] + F[3][2]*P[2][3] + P[3][3]);

    FPFT[0][0] = (FP[0][0]+FP[0][1]*F[0][1]+FP[0][2]*F[0][2]+FP[0][3]*F[0][3]);
    FPFT[0][1] = (FP[0][0]*F[1][0]+FP[0][1]+FP[0][2]*F[1][2]+FP[0][3]*F[1][3]);
    FPFT[0][2] = (FP[0][0]*F[2][0]+FP[0][1]*F[2][1]+FP[0][2]+FP[0][3]*F[2][3]);
    FPFT[0][3] = (FP[0][0]*F[3][0]+FP[0][1]*F[3][1]+FP[0][2]*F[3][2]+FP[0][3]);
    FPFT[1][0] = (FP[1][0]+FP[1][1]*F[0][1]+FP[1][2]*F[0][2]+FP[1][3]*F[0][3]);
    FPFT[1][1] = (FP[1][0]*F[1][0]+FP[1][1]+FP[1][2]*F[1][2]+FP[1][3]*F[1][3]);
    FPFT[1][2] = (FP[1][0]*F[2][0]+FP[1][1]*F[2][1]+FP[1][2]+FP[1][3]*F[2][3]);
    FPFT[1][3] = (FP[1][0]*F[3][0]+FP[1][1]*F[3][1]+FP[1][2]*F[3][2]+FP[1][3]);
    FPFT[2][0] = (FP[2][0]+FP[2][1]*F[0][1]+FP[2][2]*F[0][2]+FP[2][3]*F[0][3]);
    FPFT[2][1] = (FP[2][0]*F[1][0]+FP[2][1]+FP[2][2]*F[1][2]+FP[2][3]*F[1][3]);
    FPFT[2][2] = (FP[2][0]*F[2][0]+FP[2][1]*F[2][1]+FP[2][2]+FP[2][3]*F[2][3]);
    FPFT[2][3] = (FP[2][0]*F[3][0]+FP[2][1]*F[3][1]+FP[2][2]*F[3][2]+FP[2][3]);
    FPFT[3][0] = (FP[3][0]+FP[3][1]*F[0][1]+FP[3][2]*F[0][2]+FP[3][3]*F[0][3]);
    FPFT[3][1] = (FP[3][0]*F[1][0]+FP[3][1]+FP[3][2]*F[1][2]+FP[3][3]*F[1][3]);
    FPFT[3][2] = (FP[3][0]*F[2][0]+FP[3][1]*F[2][1]+FP[3][2]+FP[3][3]*F[2][3]);
    FPFT[3][3] = (FP[3][0]*F[3][0]+FP[3][1]*F[3][1]+FP[3][2]*F[3][2]+FP[3][3]);

    float sigmaloop2b4 = sigma*loop_time*loop_time*0.25f;
    Q[0][0] = (W[0][0]*W[0][0]+W[0][1]*W[0][1]+W[0][2]*W[0][2])*sigmaloop2b4;
    Q[0][1] = (W[0][0]*W[1][0]+W[0][1]*W[1][1]+W[0][2]*W[1][2])*sigmaloop2b4;
    Q[0][2] = (W[0][0]*W[2][0]+W[0][1]*W[2][1]+W[0][2]*W[2][2])*sigmaloop2b4;
    Q[0][3] = (W[0][0]*W[3][0]+W[0][1]*W[3][1]+W[0][2]*W[3][2])*sigmaloop2b4;
    Q[1][0] = (W[1][0]*W[0][0]+W[1][1]*W[0][1]+W[1][2]*W[0][2])*sigmaloop2b4;
    Q[1][1] = (W[1][0]*W[1][0]+W[1][1]*W[1][1]+W[1][2]*W[1][2])*sigmaloop2b4;
    Q[1][2] = (W[1][0]*W[2][0]+W[1][1]*W[2][1]+W[1][2]*W[2][2])*sigmaloop2b4;
    Q[1][3] = (W[1][0]*W[3][0]+W[1][1]*W[3][1]+W[1][2]*W[3][2])*sigmaloop2b4;
    Q[2][0] = (W[2][0]*W[0][0]+W[2][1]*W[0][1]+W[2][2]*W[0][2])*sigmaloop2b4;
    Q[2][1] = (W[2][0]*W[1][0]+W[2][1]*W[1][1]+W[2][2]*W[1][2])*sigmaloop2b4;
    Q[2][2] = (W[2][0]*W[2][0]+W[2][1]*W[2][1]+W[2][2]*W[2][2])*sigmaloop2b4;
    Q[2][3] = (W[2][0]*W[3][0]+W[2][1]*W[3][1]+W[2][2]*W[3][2])*sigmaloop2b4;
    Q[3][0] = (W[3][0]*W[0][0]+W[3][1]*W[0][1]+W[3][2]*W[0][2])*sigmaloop2b4;
    Q[3][1] = (W[3][0]*W[1][0]+W[3][1]*W[1][1]+W[3][2]*W[1][2])*sigmaloop2b4;
    Q[3][2] = (W[3][0]*W[2][0]+W[3][1]*W[2][1]+W[3][2]*W[2][2])*sigmaloop2b4;
    Q[3][3] = (W[3][0]*W[3][0]+W[3][1]*W[3][1]+W[3][2]*W[3][2])*sigmaloop2b4;

    P[0][0] = FPFT[0][0] + Q[0][0];
    P[0][1] = FPFT[0][1] + Q[0][1];
    P[0][2] = FPFT[0][2] + Q[0][2];
    P[0][3] = FPFT[0][3] + Q[0][3];
    P[1][0] = FPFT[1][0] + Q[1][0];
    P[1][1] = FPFT[1][1] + Q[1][1];
    P[1][2] = FPFT[1][2] + Q[1][2];
    P[1][3] = FPFT[1][3] + Q[1][3];
    P[2][0] = FPFT[2][0] + Q[2][0];
    P[2][1] = FPFT[2][1] + Q[2][1];
    P[2][2] = FPFT[2][2] + Q[2][2];
    P[2][3] = FPFT[2][3] + Q[2][3];
    P[3][0] = FPFT[3][0] + Q[3][0];
    P[3][1] = FPFT[3][1] + Q[3][1];
    P[3][2] = FPFT[3][2] + Q[3][2];
    P[3][3] = FPFT[3][3] + Q[3][3];
//--------------------------------------- Update --------------------------------------//
    h[0] = 2.0f*(qx*qz-qw*qy);
    h[1] = 2.0f*(qw*qx+qy*qz);
    h[2] = 2.0f*(0.5f-qx*qx-qy*qy);
    h[3] = 2.0f*(ry*(qw*qz+qx*qy)+rz*(qx*qz-qw*qy));
    h[4] = 2.0f*(ry*(0.5f-qx*qx-qz*qz)+rz*(qw*qx+qy*qz));
    h[5] = 2.0f*(ry*(qy*qz-qw*qx)+rz*(0.5f-qx*qx-qy*qy));
    v[0] = ax - h[0];
    v[1] = ay - h[1];
    v[2] = az - h[2];
    v[3] = mx - h[3];
    v[4] = my - h[4];
    v[5] = mz - h[5];
    if(my<0 && h[2]>0){ v[3] = -v[3]; }             // *compass filter singulartiy, avoids divergence of filter while pointing South [V.V.I.]*
    
    H[0][0] = -2.0f*qy;
    H[0][1] = 2.0f*qz;
    H[0][2] = -2.0f*qw;
    H[0][3] = 2.0f*qx;
    H[1][0] = 2.0f*qx;
    H[1][1] = 2.0f*qw;
    H[1][2] = 2.0f*qz;
    H[1][3] = 2.0f*qy;
    H[2][0] = 0;
    H[2][1] = -4.0f*qx;
    H[2][2] = -4.0f*qy;
    H[2][3] = 0;
    H[3][0] = -2.0f*(ry*qz-rz*qy);
    H[3][1] = 2.0f*(ry*qy+rz*qz);
    H[3][2] = 2.0f*(ry*qx-rz*qw);
    H[3][3] = 2.0f*(ry*qw+rz*qx);
    H[4][0] = 2.0f*rz*qx;
    H[4][1] = 2.0f*(-2.0f*ry*qx+rz*qw);
    H[4][2] = 2.0f*rz*qz;
    H[4][3] = 2.0f*(-2.0f*ry*qz+rz*qy);
    H[5][0] = -2.0f*ry*qx;
    H[5][1] = 2.0f*(-ry*qw-2.0f*rz*qx);
    H[5][2] = 2.0f*(ry*qz-2.0f*rz*qy);
    H[5][3] = 2.0f*ry*qy;

    PHT[0][0] = (P[0][0]*H[0][0]+P[0][1]*H[0][1]+P[0][2]*H[0][2]+P[0][3]*H[0][3]);
    PHT[0][1] = (P[0][0]*H[1][0]+P[0][1]*H[1][1]+P[0][2]*H[1][2]+P[0][3]*H[1][3]);
    PHT[0][2] = (P[0][0]*H[2][0]+P[0][1]*H[2][1]+P[0][2]*H[2][2]+P[0][3]*H[2][3]);
    PHT[0][3] = (P[0][0]*H[3][0]+P[0][1]*H[3][1]+P[0][2]*H[3][2]+P[0][3]*H[3][3]);
    PHT[0][4] = (P[0][0]*H[4][0]+P[0][1]*H[4][1]+P[0][2]*H[4][2]+P[0][3]*H[4][3]);
    PHT[0][5] = (P[0][0]*H[5][0]+P[0][1]*H[5][1]+P[0][2]*H[5][2]+P[0][3]*H[5][3]);
    PHT[1][0] = (P[1][0]*H[0][0]+P[1][1]*H[0][1]+P[1][2]*H[0][2]+P[1][3]*H[0][3]);
    PHT[1][1] = (P[1][0]*H[1][0]+P[1][1]*H[1][1]+P[1][2]*H[1][2]+P[1][3]*H[1][3]);
    PHT[1][2] = (P[1][0]*H[2][0]+P[1][1]*H[2][1]+P[1][2]*H[2][2]+P[1][3]*H[2][3]);
    PHT[1][3] = (P[1][0]*H[3][0]+P[1][1]*H[3][1]+P[1][2]*H[3][2]+P[1][3]*H[3][3]);
    PHT[1][4] = (P[1][0]*H[4][0]+P[1][1]*H[4][1]+P[1][2]*H[4][2]+P[1][3]*H[4][3]);
    PHT[1][5] = (P[1][0]*H[5][0]+P[1][1]*H[5][1]+P[1][2]*H[5][2]+P[1][3]*H[5][3]);
    PHT[2][0] = (P[2][0]*H[0][0]+P[2][1]*H[0][1]+P[2][2]*H[0][2]+P[2][3]*H[0][3]);
    PHT[2][1] = (P[2][0]*H[1][0]+P[2][1]*H[1][1]+P[2][2]*H[1][2]+P[2][3]*H[1][3]);
    PHT[2][2] = (P[2][0]*H[2][0]+P[2][1]*H[2][1]+P[2][2]*H[2][2]+P[2][3]*H[2][3]);
    PHT[2][3] = (P[2][0]*H[3][0]+P[2][1]*H[3][1]+P[2][2]*H[3][2]+P[2][3]*H[3][3]);
    PHT[2][4] = (P[2][0]*H[4][0]+P[2][1]*H[4][1]+P[2][2]*H[4][2]+P[2][3]*H[4][3]);
    PHT[2][5] = (P[2][0]*H[5][0]+P[2][1]*H[5][1]+P[2][2]*H[5][2]+P[2][3]*H[5][3]);
    PHT[3][0] = (P[3][0]*H[0][0]+P[3][1]*H[0][1]+P[3][2]*H[0][2]+P[3][3]*H[0][3]);
    PHT[3][1] = (P[3][0]*H[1][0]+P[3][1]*H[1][1]+P[3][2]*H[1][2]+P[3][3]*H[1][3]);
    PHT[3][2] = (P[3][0]*H[2][0]+P[3][1]*H[2][1]+P[3][2]*H[2][2]+P[3][3]*H[2][3]);
    PHT[3][3] = (P[3][0]*H[3][0]+P[3][1]*H[3][1]+P[3][2]*H[3][2]+P[3][3]*H[3][3]);
    PHT[3][4] = (P[3][0]*H[4][0]+P[3][1]*H[4][1]+P[3][2]*H[4][2]+P[3][3]*H[4][3]);
    PHT[3][5] = (P[3][0]*H[5][0]+P[3][1]*H[5][1]+P[3][2]*H[5][2]+P[3][3]*H[5][3]);

    uint8_t i, j, k, n;
    float a[13][13] = {0},d;
    n = 6;
    a[1][1] = (H[0][0]*PHT[0][0]+H[0][1]*PHT[1][0]+H[0][2]*PHT[2][0]+H[0][3]*PHT[3][0]) + R[0];
    a[1][2] = (H[0][0]*PHT[0][1]+H[0][1]*PHT[1][1]+H[0][2]*PHT[2][1]+H[0][3]*PHT[3][1]);  
    a[1][3] = (H[0][0]*PHT[0][2]+H[0][1]*PHT[1][2]+H[0][2]*PHT[2][2]+H[0][3]*PHT[3][2]);  
    a[1][4] = (H[0][0]*PHT[0][3]+H[0][1]*PHT[1][3]+H[0][2]*PHT[2][3]+H[0][3]*PHT[3][3]);
    a[1][5] = (H[0][0]*PHT[0][4]+H[0][1]*PHT[1][4]+H[0][2]*PHT[2][4]+H[0][3]*PHT[3][4]);  
    a[1][6] = (H[0][0]*PHT[0][5]+H[0][1]*PHT[1][5]+H[0][2]*PHT[2][5]+H[0][3]*PHT[3][5]);
    
    a[2][1] = (H[1][0]*PHT[0][0]+H[1][1]*PHT[1][0]+H[1][2]*PHT[2][0]+H[1][3]*PHT[3][0]);
    a[2][2] = (H[1][0]*PHT[0][1]+H[1][1]*PHT[1][1]+H[1][2]*PHT[2][1]+H[1][3]*PHT[3][1]) + R[0];  
    a[2][3] = (H[1][0]*PHT[0][2]+H[1][1]*PHT[1][2]+H[1][2]*PHT[2][2]+H[1][3]*PHT[3][2]);  
    a[2][4] = (H[1][0]*PHT[0][3]+H[1][1]*PHT[1][3]+H[1][2]*PHT[2][3]+H[1][3]*PHT[3][3]);
    a[2][5] = (H[1][0]*PHT[0][4]+H[1][1]*PHT[1][4]+H[1][2]*PHT[2][4]+H[1][3]*PHT[3][4]);  
    a[2][6] = (H[1][0]*PHT[0][5]+H[1][1]*PHT[1][5]+H[1][2]*PHT[2][5]+H[1][3]*PHT[3][5]);  
    
    a[3][1] = (H[2][0]*PHT[0][0]+H[2][1]*PHT[1][0]+H[2][2]*PHT[2][0]+H[2][3]*PHT[3][0]);
    a[3][2] = (H[2][0]*PHT[0][1]+H[2][1]*PHT[1][1]+H[2][2]*PHT[2][1]+H[2][3]*PHT[3][1]);  
    a[3][3] = (H[2][0]*PHT[0][2]+H[2][1]*PHT[1][2]+H[2][2]*PHT[2][2]+H[2][3]*PHT[3][2]) + R[0];  
    a[3][4] = (H[2][0]*PHT[0][3]+H[2][1]*PHT[1][3]+H[2][2]*PHT[2][3]+H[2][3]*PHT[3][3]);
    a[3][5] = (H[2][0]*PHT[0][4]+H[2][1]*PHT[1][4]+H[2][2]*PHT[2][4]+H[2][3]*PHT[3][4]);  
    a[3][6] = (H[2][0]*PHT[0][5]+H[2][1]*PHT[1][5]+H[2][2]*PHT[2][5]+H[2][3]*PHT[3][5]);
    
    a[4][1] = (H[3][0]*PHT[0][0]+H[3][1]*PHT[1][0]+H[3][2]*PHT[2][0]+H[3][3]*PHT[3][0]);
    a[4][2] = (H[3][0]*PHT[0][1]+H[3][1]*PHT[1][1]+H[3][2]*PHT[2][1]+H[3][3]*PHT[3][1]);
    a[4][3] = (H[3][0]*PHT[0][2]+H[3][1]*PHT[1][2]+H[3][2]*PHT[2][2]+H[3][3]*PHT[3][2]);
    a[4][4] = (H[3][0]*PHT[0][3]+H[3][1]*PHT[1][3]+H[3][2]*PHT[2][3]+H[3][3]*PHT[3][3]) + R[1];
    a[4][5] = (H[3][0]*PHT[0][4]+H[3][1]*PHT[1][4]+H[3][2]*PHT[2][4]+H[3][3]*PHT[3][4]);
    a[4][6] = (H[3][0]*PHT[0][5]+H[3][1]*PHT[1][5]+H[3][2]*PHT[2][5]+H[3][3]*PHT[3][5]);
    
    a[5][1] = (H[4][0]*PHT[0][0]+H[4][1]*PHT[1][0]+H[4][2]*PHT[2][0]+H[4][3]*PHT[3][0]);
    a[5][2] = (H[4][0]*PHT[0][1]+H[4][1]*PHT[1][1]+H[4][2]*PHT[2][1]+H[4][3]*PHT[3][1]);
    a[5][3] = (H[4][0]*PHT[0][2]+H[4][1]*PHT[1][2]+H[4][2]*PHT[2][2]+H[4][3]*PHT[3][2]);
    a[5][4] = (H[4][0]*PHT[0][3]+H[4][1]*PHT[1][3]+H[4][2]*PHT[2][3]+H[4][3]*PHT[3][3]);
    a[5][5] = (H[4][0]*PHT[0][4]+H[4][1]*PHT[1][4]+H[4][2]*PHT[2][4]+H[4][3]*PHT[3][4]) + R[1];  
    a[5][6] = (H[4][0]*PHT[0][5]+H[4][1]*PHT[1][5]+H[4][2]*PHT[2][5]+H[4][3]*PHT[3][5]);
    
    a[6][1] = (H[5][0]*PHT[0][0]+H[5][1]*PHT[1][0]+H[5][2]*PHT[2][0]+H[5][3]*PHT[3][0]);
    a[6][2] = (H[5][0]*PHT[0][1]+H[5][1]*PHT[1][1]+H[5][2]*PHT[2][1]+H[5][3]*PHT[3][1]);
    a[6][3] = (H[5][0]*PHT[0][2]+H[5][1]*PHT[1][2]+H[5][2]*PHT[2][2]+H[5][3]*PHT[3][2]);
    a[6][4] = (H[5][0]*PHT[0][3]+H[5][1]*PHT[1][3]+H[5][2]*PHT[2][3]+H[5][3]*PHT[3][3]);
    a[6][5] = (H[5][0]*PHT[0][4]+H[5][1]*PHT[1][4]+H[5][2]*PHT[2][4]+H[5][3]*PHT[3][4]);
    a[6][6] = (H[5][0]*PHT[0][5]+H[5][1]*PHT[1][5]+H[5][2]*PHT[2][5]+H[5][3]*PHT[3][5]) + R[1];   
    for (i = 1; i <= n; i++){
        for (j = 1; j <= 2 * n; j++){
            if (j == (i + n)){
                a[i][j] = 1;
            }
        }
    }
    for (i = n; i > 1; i--){
        if (a[i-1][1] < a[i][1]){
            for(j = 1; j <= n * 2; j++){
                d = a[i][j];
                a[i][j] = a[i-1][j];
                a[i-1][j] = d;
            }
        }
    }
    for (i = 1; i <= n; i++){
        for (j = 1; j <= n * 2; j++){
            if (j != i){
                d = a[j][i] / a[i][i];
                for (k = 1; k <= n * 2; k++){
                    a[j][k] = a[j][k] - (a[i][k] * d);
                }
            }
        }
    }
    for (i = 1; i <= n; i++){
        d=a[i][i];
        for (j = 1; j <= n * 2; j++){
            a[i][j] = a[i][j] / d;
        }
    }
    for (i = 1; i <= n; i++){
        for (j = n + 1; j <= n * 2; j++){
            S_inv[i-1][j-7] = a[i][j];
        }
    }

    K[0][0] = (PHT[0][0]*S_inv[0][0]+PHT[0][1]*S_inv[1][0]+PHT[0][2]*S_inv[2][0]+PHT[0][3]*S_inv[3][0]+PHT[0][4]*S_inv[4][0]+PHT[0][5]*S_inv[5][0]);
    K[0][1] = (PHT[0][0]*S_inv[0][1]+PHT[0][1]*S_inv[1][1]+PHT[0][2]*S_inv[2][1]+PHT[0][3]*S_inv[3][1]+PHT[0][4]*S_inv[4][1]+PHT[0][5]*S_inv[5][1]);
    K[0][2] = (PHT[0][0]*S_inv[0][2]+PHT[0][1]*S_inv[1][2]+PHT[0][2]*S_inv[2][2]+PHT[0][3]*S_inv[3][2]+PHT[0][4]*S_inv[4][2]+PHT[0][5]*S_inv[5][2]);
    K[0][3] = (PHT[0][0]*S_inv[0][3]+PHT[0][1]*S_inv[1][3]+PHT[0][2]*S_inv[2][3]+PHT[0][3]*S_inv[3][3]+PHT[0][4]*S_inv[4][3]+PHT[0][5]*S_inv[5][3]);
    K[0][4] = (PHT[0][0]*S_inv[0][4]+PHT[0][1]*S_inv[1][4]+PHT[0][2]*S_inv[2][4]+PHT[0][3]*S_inv[3][4]+PHT[0][4]*S_inv[4][4]+PHT[0][5]*S_inv[5][4]);
    K[0][5] = (PHT[0][0]*S_inv[0][5]+PHT[0][1]*S_inv[1][5]+PHT[0][2]*S_inv[2][5]+PHT[0][3]*S_inv[3][5]+PHT[0][4]*S_inv[4][5]+PHT[0][5]*S_inv[5][5]);
    K[1][0] = (PHT[1][0]*S_inv[0][0]+PHT[1][1]*S_inv[1][0]+PHT[1][2]*S_inv[2][0]+PHT[1][3]*S_inv[3][0]+PHT[1][4]*S_inv[4][0]+PHT[1][5]*S_inv[5][0]);
    K[1][1] = (PHT[1][0]*S_inv[0][1]+PHT[1][1]*S_inv[1][1]+PHT[1][2]*S_inv[2][1]+PHT[1][3]*S_inv[3][1]+PHT[1][4]*S_inv[4][1]+PHT[1][5]*S_inv[5][1]);
    K[1][2] = (PHT[1][0]*S_inv[0][2]+PHT[1][1]*S_inv[1][2]+PHT[1][2]*S_inv[2][2]+PHT[1][3]*S_inv[3][2]+PHT[1][4]*S_inv[4][2]+PHT[1][5]*S_inv[5][2]);
    K[1][3] = (PHT[1][0]*S_inv[0][3]+PHT[1][1]*S_inv[1][3]+PHT[1][2]*S_inv[2][3]+PHT[1][3]*S_inv[3][3]+PHT[1][4]*S_inv[4][3]+PHT[1][5]*S_inv[5][3]);
    K[1][4] = (PHT[1][0]*S_inv[0][4]+PHT[1][1]*S_inv[1][4]+PHT[1][2]*S_inv[2][4]+PHT[1][3]*S_inv[3][4]+PHT[1][4]*S_inv[4][4]+PHT[1][5]*S_inv[5][4]);
    K[1][5] = (PHT[1][0]*S_inv[0][5]+PHT[1][1]*S_inv[1][5]+PHT[1][2]*S_inv[2][5]+PHT[1][3]*S_inv[3][5]+PHT[1][4]*S_inv[4][5]+PHT[1][5]*S_inv[5][5]);
    K[2][0] = (PHT[2][0]*S_inv[0][0]+PHT[2][1]*S_inv[1][0]+PHT[2][2]*S_inv[2][0]+PHT[2][3]*S_inv[3][0]+PHT[2][4]*S_inv[4][0]+PHT[2][5]*S_inv[5][0]);
    K[2][1] = (PHT[2][0]*S_inv[0][1]+PHT[2][1]*S_inv[1][1]+PHT[2][2]*S_inv[2][1]+PHT[2][3]*S_inv[3][1]+PHT[2][4]*S_inv[4][1]+PHT[2][5]*S_inv[5][1]);
    K[2][2] = (PHT[2][0]*S_inv[0][2]+PHT[2][1]*S_inv[1][2]+PHT[2][2]*S_inv[2][2]+PHT[2][3]*S_inv[3][2]+PHT[2][4]*S_inv[4][2]+PHT[2][5]*S_inv[5][2]);
    K[2][3] = (PHT[2][0]*S_inv[0][3]+PHT[2][1]*S_inv[1][3]+PHT[2][2]*S_inv[2][3]+PHT[2][3]*S_inv[3][3]+PHT[2][4]*S_inv[4][3]+PHT[2][5]*S_inv[5][3]);
    K[2][4] = (PHT[2][0]*S_inv[0][4]+PHT[2][1]*S_inv[1][4]+PHT[2][2]*S_inv[2][4]+PHT[2][3]*S_inv[3][4]+PHT[2][4]*S_inv[4][4]+PHT[2][5]*S_inv[5][4]);
    K[2][5] = (PHT[2][0]*S_inv[0][5]+PHT[2][1]*S_inv[1][5]+PHT[2][2]*S_inv[2][5]+PHT[2][3]*S_inv[3][5]+PHT[2][4]*S_inv[4][5]+PHT[2][5]*S_inv[5][5]);
    K[3][0] = (PHT[3][0]*S_inv[0][0]+PHT[3][1]*S_inv[1][0]+PHT[3][2]*S_inv[2][0]+PHT[3][3]*S_inv[3][0]+PHT[3][4]*S_inv[4][0]+PHT[3][5]*S_inv[5][0]);
    K[3][1] = (PHT[3][0]*S_inv[0][1]+PHT[3][1]*S_inv[1][1]+PHT[3][2]*S_inv[2][1]+PHT[3][3]*S_inv[3][1]+PHT[3][4]*S_inv[4][1]+PHT[3][5]*S_inv[5][1]);
    K[3][2] = (PHT[3][0]*S_inv[0][2]+PHT[3][1]*S_inv[1][2]+PHT[3][2]*S_inv[2][2]+PHT[3][3]*S_inv[3][2]+PHT[3][4]*S_inv[4][2]+PHT[3][5]*S_inv[5][2]);
    K[3][3] = (PHT[3][0]*S_inv[0][3]+PHT[3][1]*S_inv[1][3]+PHT[3][2]*S_inv[2][3]+PHT[3][3]*S_inv[3][3]+PHT[3][4]*S_inv[4][3]+PHT[3][5]*S_inv[5][3]);
    K[3][4] = (PHT[3][0]*S_inv[0][4]+PHT[3][1]*S_inv[1][4]+PHT[3][2]*S_inv[2][4]+PHT[3][3]*S_inv[3][4]+PHT[3][4]*S_inv[4][4]+PHT[3][5]*S_inv[5][4]);
    K[3][5] = (PHT[3][0]*S_inv[0][5]+PHT[3][1]*S_inv[1][5]+PHT[3][2]*S_inv[2][5]+PHT[3][3]*S_inv[3][5]+PHT[3][4]*S_inv[4][5]+PHT[3][5]*S_inv[5][5]);

    Q_out[0] = (K[0][0]*v[0]+K[0][1]*v[1]+K[0][2]*v[2]+K[0][3]*v[3]+K[0][4]*v[4]+K[0][5]*v[5]);
    Q_out[1] = (K[1][0]*v[0]+K[1][1]*v[1]+K[1][2]*v[2]+K[1][3]*v[3]+K[1][4]*v[4]+K[1][5]*v[5]);
    Q_out[2] = (K[2][0]*v[0]+K[2][1]*v[1]+K[2][2]*v[2]+K[2][3]*v[3]+K[2][4]*v[4]+K[2][5]*v[5]);
    Q_out[3] = (K[3][0]*v[0]+K[3][1]*v[1]+K[3][2]*v[2]+K[3][3]*v[3]+K[3][4]*v[4]+K[3][5]*v[5]);

    I[0][0] = 1-(K[0][0]*H[0][0]+K[0][1]*H[1][0]+K[0][2]*H[2][0]+K[0][3]*H[3][0]+K[0][4]*H[4][0]+K[0][5]*H[5][0]);
    I[0][1] = -(K[0][0]*H[0][1]+K[0][1]*H[1][1]+K[0][2]*H[2][1]+K[0][3]*H[3][1]+K[0][4]*H[4][1]+K[0][5]*H[5][1]);
    I[0][2] = -(K[0][0]*H[0][2]+K[0][1]*H[1][2]+K[0][2]*H[2][2]+K[0][3]*H[3][2]+K[0][4]*H[4][2]+K[0][5]*H[5][2]);
    I[0][3] = -(K[0][0]*H[0][3]+K[0][1]*H[1][3]+K[0][2]*H[2][3]+K[0][3]*H[3][3]+K[0][4]*H[4][3]+K[0][5]*H[5][3]);
    I[1][0] = -(K[1][0]*H[0][0]+K[1][1]*H[1][0]+K[1][2]*H[2][0]+K[1][3]*H[3][0]+K[1][4]*H[4][0]+K[1][5]*H[5][0]);
    I[1][1] = 1-(K[1][0]*H[0][1]+K[1][1]*H[1][1]+K[1][2]*H[2][1]+K[1][3]*H[3][1]+K[1][4]*H[4][1]+K[1][5]*H[5][1]);
    I[1][2] = -(K[1][0]*H[0][2]+K[1][1]*H[1][2]+K[1][2]*H[2][2]+K[1][3]*H[3][2]+K[1][4]*H[4][2]+K[1][5]*H[5][2]);
    I[1][3] = -(K[1][0]*H[0][3]+K[1][1]*H[1][3]+K[1][2]*H[2][3]+K[1][3]*H[3][3]+K[1][4]*H[4][3]+K[1][5]*H[5][3]);
    I[2][0] = -(K[2][0]*H[0][0]+K[2][1]*H[1][0]+K[2][2]*H[2][0]+K[2][3]*H[3][0]+K[2][4]*H[4][0]+K[2][5]*H[5][0]);
    I[2][1] = -(K[2][0]*H[0][1]+K[2][1]*H[1][1]+K[2][2]*H[2][1]+K[2][3]*H[3][1]+K[2][4]*H[4][1]+K[2][5]*H[5][1]);
    I[2][2] = 1-(K[2][0]*H[0][2]+K[2][1]*H[1][2]+K[2][2]*H[2][2]+K[2][3]*H[3][2]+K[2][4]*H[4][2]+K[2][5]*H[5][2]);
    I[2][3] = -(K[2][0]*H[0][3]+K[2][1]*H[1][3]+K[2][2]*H[2][3]+K[2][3]*H[3][3]+K[2][4]*H[4][3]+K[2][5]*H[5][3]);
    I[3][0] = -(K[3][0]*H[0][0]+K[3][1]*H[1][0]+K[3][2]*H[2][0]+K[3][3]*H[3][0]+K[3][4]*H[4][0]+K[3][5]*H[5][0]);
    I[3][1] = -(K[3][0]*H[0][1]+K[3][1]*H[1][1]+K[3][2]*H[2][1]+K[3][3]*H[3][1]+K[3][4]*H[4][1]+K[3][5]*H[5][1]);
    I[3][2] = -(K[3][0]*H[0][2]+K[3][1]*H[1][2]+K[3][2]*H[2][2]+K[3][3]*H[3][2]+K[3][4]*H[4][2]+K[3][5]*H[5][2]);
    I[3][3] = 1-(K[3][0]*H[0][3]+K[3][1]*H[1][3]+K[3][2]*H[2][3]+K[3][3]*H[3][3]+K[3][4]*H[4][3]+K[3][5]*H[5][3]);
    
    I_KHP[0][0] = (I[0][0]*P[0][0]+I[0][1]*P[1][0]+I[0][2]*P[2][0]+I[0][3]*P[3][0]);
    I_KHP[0][1] = (I[0][0]*P[0][1]+I[0][1]*P[1][1]+I[0][2]*P[2][1]+I[0][3]*P[3][1]);
    I_KHP[0][2] = (I[0][0]*P[0][2]+I[0][1]*P[1][2]+I[0][2]*P[2][2]+I[0][3]*P[3][2]);
    I_KHP[0][3] = (I[0][0]*P[0][3]+I[0][1]*P[1][3]+I[0][2]*P[2][3]+I[0][3]*P[3][3]);
    I_KHP[1][0] = (I[1][0]*P[0][0]+I[1][1]*P[1][0]+I[1][2]*P[2][0]+I[1][3]*P[3][0]);
    I_KHP[1][1] = (I[1][0]*P[0][1]+I[1][1]*P[1][1]+I[1][2]*P[2][1]+I[1][3]*P[3][1]);
    I_KHP[1][2] = (I[1][0]*P[0][2]+I[1][1]*P[1][2]+I[1][2]*P[2][2]+I[1][3]*P[3][2]);
    I_KHP[1][3] = (I[1][0]*P[0][3]+I[1][1]*P[1][3]+I[1][2]*P[2][3]+I[1][3]*P[3][3]);
    I_KHP[2][0] = (I[2][0]*P[0][0]+I[2][1]*P[1][0]+I[2][2]*P[2][0]+I[2][3]*P[3][0]);
    I_KHP[2][1] = (I[2][0]*P[0][1]+I[2][1]*P[1][1]+I[2][2]*P[2][1]+I[2][3]*P[3][1]);
    I_KHP[2][2] = (I[2][0]*P[0][2]+I[2][1]*P[1][2]+I[2][2]*P[2][2]+I[2][3]*P[3][2]);
    I_KHP[2][3] = (I[2][0]*P[0][3]+I[2][1]*P[1][3]+I[2][2]*P[2][3]+I[2][3]*P[3][3]);
    I_KHP[3][0] = (I[3][0]*P[0][0]+I[3][1]*P[1][0]+I[3][2]*P[2][0]+I[3][3]*P[3][0]);
    I_KHP[3][1] = (I[3][0]*P[0][1]+I[3][1]*P[1][1]+I[3][2]*P[2][1]+I[3][3]*P[3][1]);
    I_KHP[3][2] = (I[3][0]*P[0][2]+I[3][1]*P[1][2]+I[3][2]*P[2][2]+I[3][3]*P[3][2]);
    I_KHP[3][3] = (I[3][0]*P[0][3]+I[3][1]*P[1][3]+I[3][2]*P[2][3]+I[3][3]*P[3][3]);

    P[0][0] = I_KHP[0][0];
    P[0][1] = I_KHP[0][1];
    P[0][2] = I_KHP[0][2];
    P[0][3] = I_KHP[0][3];
    P[1][0] = I_KHP[1][0];
    P[1][1] = I_KHP[1][1];
    P[1][2] = I_KHP[1][2];
    P[1][3] = I_KHP[1][3];
    P[2][0] = I_KHP[2][0];
    P[2][1] = I_KHP[2][1];
    P[2][2] = I_KHP[2][2];
    P[2][3] = I_KHP[2][3];
    P[3][0] = I_KHP[3][0];
    P[3][1] = I_KHP[3][1];
    P[3][2] = I_KHP[3][2];
    P[3][3] = I_KHP[3][3];

    qw = qw + Q_out[0];
    qx = qx + Q_out[1];
    qy = qy + Q_out[2];
    qz = qz + Q_out[3];
    norm_q = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    qw = qw/norm_q;
    qx = qx/norm_q;
    qy = qy/norm_q;
    qz = qz/norm_q;

    float q_dot[4];

    q_dot[0] = (qw - q_prev[0])/loop_time;
    q_dot[1] = (qx - q_prev[1])/loop_time;
    q_dot[2] = (qy - q_prev[2])/loop_time;
    q_dot[3] = (qz - q_prev[3])/loop_time;

    wx_crct = 2*57.2958f*( - q_prev[1]*q_dot[0] + q_prev[0]*q_dot[1] + q_prev[3]*q_dot[2] - q_prev[2]*q_dot[3] );
    wy_crct = 2*57.2958f*( - q_prev[2]*q_dot[0] - q_prev[3]*q_dot[1] + q_prev[0]*q_dot[2] + q_prev[1]*q_dot[3] );
    wz_crct = 2*57.2958f*( - q_prev[3]*q_dot[0] + q_prev[2]*q_dot[1] - q_prev[1]*q_dot[2] + q_prev[0]*q_dot[3] );


    float temp_angle_half, q[4], q1,q2;
    temp_angle_half = ( roll_offset_angle/2.0f ) * 0.0174532925f;
    q_level_rot[0] = cosf( temp_angle_half ); q_level_rot[2] = sinf( temp_angle_half );
    temp_angle_half = ( pitch_offset_angle/2.0f ) * 0.0174532925f;
    q1 = cosf( temp_angle_half ); q2 = sinf( temp_angle_half );
    q[0] = q1*q_level_rot[0]; q[1] = q2*q_level_rot[0]; q[2] = q1*q_level_rot[2]; q[3] = q2*q_level_rot[2];
    q_level_rot[0] = q[0]; q_level_rot[1] = q[1]; q_level_rot[2] = q[2]; q_level_rot[3] = q[3];

    q_leveled[0] = (q_level_rot[0]*qw - q_level_rot[1]*qx - q_level_rot[2]*qy - q_level_rot[3]*qz);
    q_leveled[1] = (q_level_rot[0]*qx + q_level_rot[1]*qw + q_level_rot[2]*qz - q_level_rot[3]*qy);
    q_leveled[2] = (q_level_rot[0]*qy - q_level_rot[1]*qz + q_level_rot[2]*qw + q_level_rot[3]*qx);
    q_leveled[3] = (q_level_rot[0]*qz + q_level_rot[1]*qy - q_level_rot[2]*qx + q_level_rot[3]*qw);

    yaw_rad = atan2f( 2*(q_leveled[0]*q_leveled[3] + q_leveled[1]*q_leveled[2]) , (1 - 2*(q_leveled[2]*q_leveled[2] + q_leveled[3]*q_leveled[3])) );
    
    q_prev[0]=qw; q_prev[1]=qx; q_prev[2]=qy; q_prev[3]=qz; // update prev quaternion
}

static void PID_angular_rates_ctrl(){
    twoX_P_gain = 1.82921f - 0.0003f*ctrl_channel[2];
    twoX_D_gain = 0.03f - 0.0000026f*ctrl_channel[2];

    if(is_GPS_mode_ON == false){
        ctrl_roll  = (float)(ctrl_channel[0]-50) * 0.5f;
        ctrl_pitch = (float)(50-ctrl_channel[1]) * 0.5f;
    }
    else if(is_GPS_mode_ON == true){
        ctrl_roll  = (float)(GPHC_roll_angle_out_body) * 0.5f;
        ctrl_pitch = (float)(-GPHC_pitch_angle_out_body) * 0.5f;
    }

    if( ctrl_channel[2] == 0 ){ ctrl_yaw = 57.2957795f*yaw_rad; }
    ctrl_yaw += (float)(50-ctrl_channel[3]) * 0.008; // inc or dec the constant value for fast or slow yaw rates.

    Quaternion qf = get_Quaternion_from_bodyframe_angles( ctrl_roll, ctrl_pitch, ctrl_yaw);
    Quaternion q_error = q1_dot_q2( q_leveled[0], -q_leveled[1], -q_leveled[2], -q_leveled[3],    qf.w, qf.x, qf.y, qf.z );
    Angles error_angles = get_error_angles_from_Quaternion(quat_inv(q_error));

    //---------------------------------------- Roll PID --------------------------------------//
    desired_angular_rate[0] = - error_angles.roll;

    if( desired_angular_rate[0] > -11.0f || desired_angular_rate[0] < 11.0f ){ desired_angular_rate[0] = desired_angular_rate[0] * angle_to_rate_gain; }
    else if( desired_angular_rate[0] >= 11.0f ){ desired_angular_rate[0] =   ( 13.0f + 10.0f * sqrtf(   desired_angular_rate[0] -  7.0f) ); }
    else if( desired_angular_rate[0] < -11.0f ){ desired_angular_rate[0] = - ( 13.0f + 10.0f * sqrtf( - desired_angular_rate[0] -  7.0f) ); }

    error[0] = desired_angular_rate[0] -wy_crct;
    setpoint_free_error[0] = -wy_crct;
    pid_Integral[0] += twoX_I_gain * loop_time * ( prev_error[0] + error[0] )/2.0f;
    if ( pid_Integral[0]*error[0] < 0){ pid_Integral[0] = 0; }
    pid_Derivative[0] =  0.7f*pid_Derivative[0] + 0.3f*twoX_D_gain*( setpoint_free_error[0] - setpoint_free_prev_error[0] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Roll_PID  = int(  twoX_P_gain*error[0] + pid_Integral[0] + pid_Derivative[0] );
    if( Roll_PID < -Roll_PID_lim ){ Roll_PID = -Roll_PID_lim; }  else if( Roll_PID > Roll_PID_lim ){ Roll_PID = Roll_PID_lim; }
    prev_error[0] = error[0];
    setpoint_free_prev_error[0] = setpoint_free_error[0];
    //-----------------------------------------------------------------------------------------//

    //---------------------------------------- Pitch PID --------------------------------------//
    desired_angular_rate[1] = - error_angles.pitch;

    if( desired_angular_rate[1] > -11.0f || desired_angular_rate[1] < 11.0f ){ desired_angular_rate[1] = desired_angular_rate[1] * angle_to_rate_gain; }
    else if( desired_angular_rate[1] >= 11.0f ){ desired_angular_rate[1] =   ( 13.0f + 10.0f * sqrtf(   desired_angular_rate[1] -  7.0f) ); }
    else if( desired_angular_rate[1] < -11.0f ){ desired_angular_rate[1] = - ( 13.0f + 10.0f * sqrtf( - desired_angular_rate[1] -  7.0f) ); }

    error[1] = desired_angular_rate[1] -wx_crct;
    setpoint_free_error[1] = -wx_crct;
    pid_Integral[1] += twoX_I_gain * loop_time * ( prev_error[1] + error[1] )/2.0f;
    if ( pid_Integral[1]*error[1] < 0){ pid_Integral[1] = 0; }
    pid_Derivative[1] =  0.7f*pid_Derivative[1] + 0.3f*twoX_D_gain*( setpoint_free_error[1] - setpoint_free_prev_error[1] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Pitch_PID  = int(  twoX_P_gain*error[1] + pid_Integral[1] + pid_Derivative[1] );
    if( Pitch_PID < -Pitch_PID_lim ){ Pitch_PID = -Pitch_PID_lim; }  else if( Pitch_PID > Pitch_PID_lim ){ Pitch_PID = Pitch_PID_lim; }
    prev_error[1] = error[1];
    setpoint_free_prev_error[1] = setpoint_free_error[1];
    //----------------------------------------------------------------------------------------//

    //---------------------------------------- Yaw PID ---------------------------------------//
    desired_angular_rate[2] =  - error_angles.yaw * 4.0f;
    error[2] = desired_angular_rate[2] -wz_crct;
    setpoint_free_error[2] = -wz_crct;
    pid_Integral[2] += Yaw_I_gain * loop_time * ( prev_error[2] + error[2] )/2.0f;
    if ( pid_Integral[2]*error[2] < 0){ pid_Integral[2] = 0; }
    pid_Derivative[2] =  0.7f*pid_Derivative[2] + 0.3f*Yaw_D_gain*( setpoint_free_error[2] - setpoint_free_prev_error[2] )*200.0f;  //200 is used insteaad of 1/looptime to avoid time noise
    Yaw_PID  = int(  Yaw_P_gain*error[2] + pid_Integral[2] + pid_Derivative[2] );
    if( Yaw_PID < -Yaw_PID_lim ){ Yaw_PID = -Yaw_PID_lim; }  else if( Yaw_PID > Yaw_PID_lim ){ Yaw_PID = Yaw_PID_lim; }
    prev_error[2] = error[2];
    setpoint_free_prev_error[2] = setpoint_free_error[2];
    //----------------------------------------------------------------------------------------//

}

static void GPS_Position_Hold(){
    if( is_GPS_mode_ON == false ){ is_GPS_Hold_once_run_done = false; }


    if( gps_decode_loop_shape_count == 0 && is_GPS_mode_ON == true ){

        LAT_DEG_TO_METERS = 111132.954 - 559.82 * cos(2.0*Vehicle_Lattitude*0.01745329251) + 1.175 * cos(4.0*Vehicle_Lattitude*0.01745329251) - 0.0023 * cos(6.0*Vehicle_Lattitude*0.01745329251);
        LON_DEG_TO_METERS = 111132.954 * cos(Vehicle_Lattitude*0.01745329251) - 93.5*cos(3.0*Vehicle_Lattitude*0.01745329251) + 0.118*cos(5.0*Vehicle_Lattitude*0.01745329251);

        velocity_vector_north_frame[0] = 0.15f*velocity_vector_north_frame[0] + 0.85f *  (float)( (Vehicle_Longitude - Vehicle_Longitude_Prev)* LON_DEG_TO_METERS )*10.0f;      // 10Hz loop freq
        velocity_vector_north_frame[1] = 0.15f*velocity_vector_north_frame[1] + 0.85f *  (float) ( (Vehicle_Lattitude - Vehicle_Lattitude_Prev)* LAT_DEG_TO_METERS )*10.0f;

        if( is_GPS_Hold_once_run_done == false ){     //safely set velocity zero for first time run only. Just to avoid big velocity error when current_coordinates are huge and prevs are 0.
            velocity_vector_north_frame[0] = 0; velocity_vector_north_frame[1] = 0;
            Vehicle_desired_Lattitude = Vehicle_Lattitude;  Vehicle_desired_Longitude = Vehicle_Longitude;
            Vehicle_Lattitude_Prev = Vehicle_Lattitude;  Vehicle_Longitude_Prev = Vehicle_Longitude;
            GPHC_roll_I = 0;  GPHC_pitch_I = 0;
            is_GPS_Hold_once_run_done = true;
        }

        float longitude_inc_body = ( (float)(ctrl_channel[0]-50) * ctrl_stick_to_velocity_div_gain * 0.1f ) / LON_DEG_TO_METERS;  // 0.1f loop time in seconds.;
        float lattitude_inc_body = ( (float)(ctrl_channel[1]-50) * ctrl_stick_to_velocity_div_gain * 0.1f ) / LAT_DEG_TO_METERS;

        Vehicle_desired_Longitude += ( longitude_inc_body*cosf(yaw_rad) - lattitude_inc_body*sinf(yaw_rad) );     // body to north frame conversion & add.
        Vehicle_desired_Lattitude += ( longitude_inc_body*sinf(yaw_rad) + lattitude_inc_body*cosf(yaw_rad) );

        v_p_n_f_error[0] = ( Vehicle_desired_Longitude - Vehicle_Longitude ) * LON_DEG_TO_METERS;
        v_p_n_f_error[1] = ( Vehicle_desired_Lattitude - Vehicle_Lattitude ) * LAT_DEG_TO_METERS;

        GPHC_roll_P =  GPHC_P_gain * v_p_n_f_error[0];
        GPHC_pitch_P = GPHC_P_gain * v_p_n_f_error[1];
        
        GPHC_roll_I  += GPHC_I_gain * v_p_n_f_error[0] * 0.1f;       // GPHC loop time 0.1f Seconds 
        GPHC_pitch_I += GPHC_I_gain * v_p_n_f_error[1] * 0.1f;       // GPHC loop time 0.1f Seconds
        
        if ( GPHC_roll_I > 15.0f ){ GPHC_roll_I  = 15.0f; }  else if (  GPHC_roll_I < -15.0f ){ GPHC_roll_I  = -15.0f; }
        if ( GPHC_pitch_I > 15.0f ){ GPHC_pitch_I  = 15.0f; }  else if (  GPHC_pitch_I < -15.0f ){ GPHC_pitch_I  = -15.0f; }

        GPHC_roll_D =  GPHC_D_gain * velocity_vector_north_frame[0];
        GPHC_pitch_D = GPHC_D_gain * velocity_vector_north_frame[1];

        GPHC_roll_angle_out_north =  GPHC_roll_P  + GPHC_roll_I  - GPHC_roll_D;
        GPHC_pitch_angle_out_north = GPHC_pitch_P + GPHC_pitch_I - GPHC_pitch_D;

        GPHC_roll_angle_out_body  = ( GPHC_roll_angle_out_north*cosf(-yaw_rad) - GPHC_pitch_angle_out_north*sinf(-yaw_rad) );
        GPHC_pitch_angle_out_body = ( GPHC_roll_angle_out_north*sinf(-yaw_rad) + GPHC_pitch_angle_out_north*cosf(-yaw_rad) );

        if(GPHC_roll_angle_out_body > GPHC_Vehicle_MAX_Lean_Angle){ GPHC_roll_angle_out_body = GPHC_Vehicle_MAX_Lean_Angle; }
        else if(GPHC_roll_angle_out_body < -GPHC_Vehicle_MAX_Lean_Angle){ GPHC_roll_angle_out_body = -GPHC_Vehicle_MAX_Lean_Angle; }

        if(GPHC_pitch_angle_out_body > GPHC_Vehicle_MAX_Lean_Angle){ GPHC_pitch_angle_out_body = GPHC_Vehicle_MAX_Lean_Angle; }    
        else if(GPHC_pitch_angle_out_body < -GPHC_Vehicle_MAX_Lean_Angle){ GPHC_pitch_angle_out_body = -GPHC_Vehicle_MAX_Lean_Angle; }


        //================================================================//
        out_status[0] = (uint8_t) ( (int)(velocity_vector_north_frame[0]*10.0f) + 128 );
        out_status[1] = (uint8_t) ( (int)(velocity_vector_north_frame[1]*10.0f) + 128 );
        //================================================================//

        Vehicle_Lattitude_Prev = Vehicle_Lattitude;  Vehicle_Longitude_Prev = Vehicle_Longitude;
    }
}

static void Motor_Drive(){
    motor_out[0] = ctrl_channel[2] + Roll_PID + Pitch_PID + Yaw_PID;
    motor_out[1] = ctrl_channel[2] - Roll_PID + Pitch_PID - Yaw_PID;
    motor_out[2] = ctrl_channel[2] - Roll_PID - Pitch_PID + Yaw_PID;
    motor_out[3] = ctrl_channel[2] + Roll_PID - Pitch_PID - Yaw_PID;

    if(motor_out[0] > 1000){ motor_out[0] = 1000; }
    else if(motor_out[0] < 50){ motor_out[0] = 50; }
    if(motor_out[1] > 1000){ motor_out[1] = 1000; }
    else if(motor_out[1] < 50){ motor_out[1] = 50; }
    if(motor_out[2] > 1000){ motor_out[2] = 1000; }
    else if(motor_out[2] < 50){ motor_out[2] = 50; }
    if(motor_out[3] > 1000){ motor_out[3] = 1000; }
    else if(motor_out[3] < 50){ motor_out[3] = 50; }
    if(ctrl_channel[2] == 0){ motor_out[0] = 0; motor_out[1] = 0; motor_out[2] = 0; motor_out[3] = 0; pid_Integral[0]=0; pid_Integral[1]=0; pid_Integral[2]=0; }

    PWM_Write();
}

static void Tx_Rx_Update_Variables(){

    //----------INPUT----------//
    if(uart0_in_buff[0] == '$' && uart0_in_buff[12] == '*'){
        ctrl_channel[0] = uart0_in_buff[1];
        ctrl_channel[1] = uart0_in_buff[2];
        ctrl_channel[2] = uart0_in_buff[3]*10;
        ctrl_channel[3] = uart0_in_buff[4];

        in_cmd[0] = uart0_in_buff[5];
        in_cmd[1] = uart0_in_buff[6];

        roll_offset_angle  = (float)uart0_in_buff[10] - 25.0f;
        pitch_offset_angle = (float)uart0_in_buff[11] - 25.0f;

        //-- ========= CMD_Input ========= --//
        
        if( in_cmd[0] == 'M' && in_cmd[1] == 'G' ){ is_GPS_mode_ON = true; } 
        else if( in_cmd[0] == 'M' && in_cmd[1] == 'N' ){ is_GPS_mode_ON = false; }
        
        //-- ============================= --//
    }
    //-------------------------//

    //---------OUTPUT----------//
    uart0_out_buff[0] = '$';
        
    uart0_out_buff[1] = int(q_leveled[0]*100.0f) + 100;
    uart0_out_buff[2] = int(q_leveled[1]*100.0f) + 100;
    uart0_out_buff[3] = int(q_leveled[2]*100.0f) + 100;
    uart0_out_buff[4] = int(q_leveled[3]*100.0f) + 100;


    if(Fix_type == '1'){
        for (uint8_t i = 5; i < 31; i++){ uart0_out_buff[i] = '0'; }
        uart0_out_buff[9] = '.'; uart0_out_buff[20] = '.';
        uart0_out_buff[26] = '1'; uart0_out_buff[27] = 0;
    }
    else if(Fix_type == '2' || Fix_type == '3'){
        for (uint8_t i = 5; i < 15; i++){ uart0_out_buff[i] = Lattitude[i-5]; }
        for (uint8_t i = 15; i < 26; i++){ uart0_out_buff[i] = Longitude[i-15]; }
        uart0_out_buff[26] = Fix_type;
        uart0_out_buff[27] = Sat_count;
        uart0_out_buff[28] = HDOP[0]; uart0_out_buff[29] = HDOP[1]; uart0_out_buff[30] = HDOP[2];
    }

    uart0_out_buff[31] = out_status[0]; uart0_out_buff[32] = out_status[1];

    uart0_out_buff[33] = '*';
    //-------------------------//

}

static void phrase_and_set_Calibrated_Values(){
    uint8_t floatarr_index=0,start_index=0;
    for(int i=0; i<compass_Cal_buff_max_expected_len; i++){
        if(compassCalHoldBuff[i]==','){
            int tempChr_len = i-start_index;
            char temp[tempChr_len];
            for(uint8_t j=0; j<tempChr_len; j++){ temp[j] = compassCalHoldBuff[start_index+j];}
            cal_arr[floatarr_index] = atof(temp);
            floatarr_index++;
            start_index = i+1;
            if(floatarr_index==9){break;}
        }
    }

    mag_cal[0][0] = cal_arr[0]; mag_cal[1][1] = cal_arr[1]; mag_cal[2][2] = cal_arr[2];
    mag_cal[0][1] = cal_arr[3]; mag_cal[1][0] = cal_arr[3];
    mag_cal[2][0] = cal_arr[4]; mag_cal[0][2] = cal_arr[4];
    mag_cal[1][2] = cal_arr[5]; mag_cal[2][1] = cal_arr[5];
    compass_offset_x = cal_arr[6];
    compass_offset_y = cal_arr[7];
    compass_offset_z = cal_arr[8];


}

int main() {
    
    sleep_ms(2000);
    Led_init();

    PWM_out_init();
    PWM_Write();

    UART1_setup(9600);
    GPS_init();
    DMA0_configure();
    UART0_setup(921600);

    gpio_put(PICO_DEFAULT_LED_PIN,1);
    while(!isCompassCalibrated);
    phrase_and_set_Calibrated_Values();
    gpio_put(PICO_DEFAULT_LED_PIN,0);

    I2C_Init();
    mpu6050_init();
    qmc5883_init();
    
    EKF_Init();

    while(1){
        timePrev = time;
        time = time_us_32();
        loop_time = (time - timePrev)/1000000.0f;

        IMU_Read();
        EKF_Run();

        GPS_decode();

        GPS_Position_Hold();
        
        Is_uart0_receiving_and_Action();
        Led_set();

        Tx_Rx_Update_Variables();

        PID_angular_rates_ctrl();

        Motor_Drive();

        while((time_us_32() - time) < Main_Loop_Time_MICRO_SECONDS);


    }
}