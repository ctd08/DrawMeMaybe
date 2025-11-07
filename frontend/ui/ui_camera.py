import streamlit as st

def show_camera_stage():
    # Style the actual camera widget container as the frame
    st.markdown("""
    <style>
      /* Center the camera widget on the page */
      .camera-wrap {
        display: flex;
        justify-content: center;
        align-items: center;
        margin: 1.25rem auto 0;
        padding: 0;
        width: 100%;
      }

      /* The Streamlit camera widget container */
      div[data-testid="stCameraInput"] {
        width: min(92vw, 640px);
        aspect-ratio: 4 / 5;                 /* portrait frame */
        margin: 0 auto;
        border: 16px solid #4a3b2e;          /* warm brown “frame” */
        border-radius: 24px;
        box-shadow: 0 10px 30px rgba(0,0,0,.15);
        overflow: hidden;                    /* clip edges */
        background: #f8f0df;
        display: grid;
        place-items: center;
      }

      /* Hide default label area */
      div[data-testid="stCameraInput"] label { display: none !important; }

      /* Make the live preview fill the frame nicely */
      div[data-testid="stCameraInput"] video,
      div[data-testid="stCameraInput"] canvas,
      div[data-testid="stCameraInput"] img {
        width: 100% !important;
        height: 100% !important;
        object-fit: cover !important;        /* fill frame without black bars */
      }

      /* On iOS, reduce double-tap zoom artifacts */
      .no-zoom {
        touch-action: manipulation;
        -webkit-tap-highlight-color: transparent;
      }
    </style>
    """, unsafe_allow_html=True)

    st.title("📸 Position yourself in the frame")
    st.caption("Tip: face centered, good lighting, hold the tablet upright.")

    st.markdown('<div class="camera-wrap no-zoom">', unsafe_allow_html=True)
    photo = st.camera_input("Camera", label_visibility="collapsed")
    st.markdown('</div>', unsafe_allow_html=True)

    # If you show st.image(photo), it will appear below the frame.
    # We skip that so the only visible preview is the live one inside the frame.

    col1, col2 = st.columns(2)
    with col1:
        use_ok = st.button("✅ Use this photo", use_container_width=True, disabled=(photo is None))
        if use_ok and photo is not None:
            st.session_state.camera_photo_bytes = photo.getvalue()
            st.session_state.photo_captured = True
            st.rerun()
    with col2:
        st.button("↩️ Retake", use_container_width=True)  # no-op, keeps the live view up
