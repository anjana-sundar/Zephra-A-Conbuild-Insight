import streamlit as st
from ultralytics import YOLO
import numpy as np
from PIL import Image

# Page config
st.set_page_config(
    page_title="ZEPHRA AI Monitoring",
    page_icon="🏗",
    layout="wide"
)

# Custom CSS
st.markdown("""
<style>

body{
background-color:#0e1117;
}

.title{
font-size:45px;
font-weight:bold;
text-align:center;
color:#f5a623;
}

.subtitle{
text-align:center;
color:gray;
margin-bottom:20px;
}

.card{
background-color:#1c1c1c;
padding:20px;
border-radius:15px;
box-shadow:0px 4px 12px rgba(0,0,0,0.5);
}

</style>
""", unsafe_allow_html=True)

# Header
st.markdown('<p class="title">🏗 ZEPHRA Construction AI</p>', unsafe_allow_html=True)
st.markdown('<p class="subtitle">AI-Powered Construction Monitoring & Safety System</p>', unsafe_allow_html=True)

# Sidebar
st.sidebar.title("⚙ System Settings")

model_choice = st.sidebar.selectbox(
    "Select Detection Model",
    ["best.pt", "yolov8n.pt"]
)

confidence = st.sidebar.slider(
    "Detection Confidence",
    0.0,1.0,0.25
)

# Load model
model = YOLO(model_choice)

st.markdown("### Upload Construction Image")

uploaded_file = st.file_uploader(
    "Upload site image (cracks, defects, workers etc)",
    type=["jpg","png","jpeg"]
)

if uploaded_file is not None:

    image = Image.open(uploaded_file)
    img = np.array(image)

    col1, col2 = st.columns(2)

    with col1:
        st.markdown("### Original Image")
        st.image(image, use_container_width=True)

    results = model(img, conf=confidence)
    result_img = results[0].plot()

    with col2:
        st.markdown("### AI Detection Result")
        st.image(result_img, use_container_width=True)

st.markdown("---")

# Info section
colA,colB,colC = st.columns(3)

with colA:
    st.metric("Worker Safety", "Monitoring")

with colB:
    st.metric("Defect Detection", "Active")

with colC:
    st.metric("AI Monitoring", "Real-time")

st.markdown("---")

st.markdown("""
### About ZEPHRA

**ZEPHRA** is an AI-powered system designed to monitor construction projects in real time.  
It helps detect structural defects like **cracks, spalling, efflorescence, and delamination** while also ensuring worker safety compliance.

Key Features:
- Drone-based construction monitoring
- AI detection of structural defects
- Worker PPE compliance detection
- Real-time safety alerts
- Construction progress verification
""")