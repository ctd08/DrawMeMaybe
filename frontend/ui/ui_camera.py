import streamlit as st

def show_camera_stage():
    ss = st.session_state
    ss.setdefault("cam_key", 0)
    ss.setdefault("armed", False)                # set after user clicks Enable
    ss.setdefault("pending_photo_bytes", None)   # captured but not accepted yet
    ss.setdefault("photo_captured", False)       # True only after Continue (app.py routes to chat)
    ss.setdefault("diag", {})                    # diagnostic snapshots
    ss.setdefault("fallback_basic_widget", False)# use st.camera_input fallback

    st.subheader("📸 Position yourself in the frame")
    st.caption("Tip: keep your face centered & upright; we auto-capture when aligned.")
    st.write("")

    # --- Minimal styling for the REAL <video> (so it sits inside the frame & fills it) ---
    video_id = f"camera-video-{ss.cam_key}"
    st.markdown(f"""
    <style>
      :root {{ --aspect-x: 4; --aspect-y: 5; }}
      @media (orientation: landscape) {{ :root {{ --aspect-x: 16; --aspect-y: 9; }} }}
      #{video_id} {{
        display:block;
        width:min(92vw, 640px);
        aspect-ratio:var(--aspect-x)/var(--aspect-y);
        height:auto;
        object-fit:cover; object-position:center;
        border:16px solid #4a3b2e; border-radius:24px;
        box-shadow:0 10px 30px rgba(0,0,0,.15);
        background:#f8f0df; margin:.75rem auto 0;
      }}
      @media (orientation: landscape) {{
        #{video_id} {{ width:min(92vw, 960px); }}
      }}
      .stButton > button {{ width:100%; }}
      .stImage img {{ width:100% !important; height:auto !important; }}
    </style>
    """, unsafe_allow_html=True)

    # --- Fallback path: use Streamlit's basic camera widget (no WebRTC) ---
    if ss.fallback_basic_widget:
        st.info("Fallback mode: using basic camera widget.")
        photo = st.camera_input("Camera (fallback)", label_visibility="collapsed")
        c1, c2 = st.columns(2)
        with c1:
            cont_clicked = st.button("Continue", disabled=(photo is None))
        with c2:
            retake_clicked = st.button("Retake")

        if photo is not None:
            st.success("✅ Photo captured! Review below.")
            st.image(photo, caption="Aligned capture")

        if cont_clicked and photo is not None:
            ss.camera_photo_bytes = photo.getvalue()
            ss.photo_captured = True
            st.rerun()

        if retake_clicked:
            ss.fallback_basic_widget = False
            ss.cam_key += 1
            st.rerun()
        return

    # --- Require a user gesture before getUserMedia (mobile/Edge policy) ---
    if not ss.armed:
        st.warning("Click below to enable your camera.")
        if st.button("Enable camera"):
            ss.armed = True
            ss.cam_key += 1
            st.rerun()
        return

    # --- Imports ---
    try:
        import time
        import cv2
        from typing import Any
        import av
        from av.video.frame import VideoFrame as AVVideoFrame
        from mediapipe.python.solutions import face_mesh as mp_face_mesh
        from streamlit_webrtc import (
            webrtc_streamer, WebRtcMode, RTCConfiguration, VideoProcessorBase
        )
    except Exception as e:
        st.error(f"Import error: {type(e).__name__}: {e}")
        st.info("Install/verify: streamlit-webrtc==0.47.7, mediapipe, av, aiortc, opencv-python.")
        return

    rtc_config = RTCConfiguration({"iceServers": [{"urls": ["stun:stun.l.google.com:19302"]}]})

    # Start with VERY permissive constraints; we’re on a laptop now.
    constraints = {"video": True, "audio": False}
    video_attrs = {
        "id": video_id,
        "playsinline": True, "muted": True, "controls": False,
        "autoPlay": True, "style": {"display": "block"},
    }

    # --- Processor with basic alignment + diagnostics ---
    class LiveAlign(VideoProcessorBase):
        def __init__(self):
            self.mesh = mp_face_mesh.FaceMesh(
                static_image_mode=False, max_num_faces=1, refine_landmarks=True,
                min_detection_confidence=0.5, min_tracking_confidence=0.5
            )
            self.frames_ok = 0
            self.captured = None     # BGR ndarray
            self.freeze = False
            self.last_frame_at = 0.0

        def recv(self, frame):  # type: ignore[override]
            self.last_frame_at = time.time()

            if self.freeze and self.captured is not None:
                return AVVideoFrame.from_ndarray(self.captured, format="bgr24")

            img = frame.to_ndarray(format="bgr24")
            h, w = img.shape[:2]

            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            res: Any = self.mesh.process(rgb)
            lms = getattr(res, "multi_face_landmarks", None)

            text, ok = "No face detected", False
            if lms:
                lm = lms[0].landmark
                left_eye = lm[33]; right_eye = lm[263]
                cx = int((left_eye.x + right_eye.x) * 0.5 * w)
                cy = int((left_eye.y + right_eye.y) * 0.5 * h)
                fx, fy = w // 2, h // 2
                dist = ((cx - fx)**2 + (cy - fy)**2) ** 0.5
                ok = dist < 60

                # Guides
                import cv2 as _cv
                _cv.rectangle(img, (w//2-120, h//2-160), (w//2+120, h//2+160), (0, 200, 0), 2)
                _cv.circle(img, (fx, fy), 4, (255, 255, 255), -1)
                text = "Perfect! Hold still..." if ok else "Center your head"

            self.frames_ok = self.frames_ok + 1 if ok else 0
            if self.frames_ok > 10 and self.captured is None:
                self.captured = img.copy()
                self.freeze = True

            import cv2 as _cv
            _cv.rectangle(img, (0, 0), (w, 42), (0, 0, 0), -1)
            _cv.putText(img, text, (12, 28), _cv.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if ok else (60, 200, 255), 2, _cv.LINE_AA)
            return AVVideoFrame.from_ndarray(img, format="bgr24")

    # --- Mount WebRTC ---
    ctx = webrtc_streamer(
        key=f"camera-{ss.cam_key}",
        mode=WebRtcMode.SENDRECV,
        rtc_configuration=rtc_config,
        media_stream_constraints=constraints,
        video_processor_factory=LiveAlign,
        video_html_attrs=video_attrs,
        desired_playing_state=True,
        async_processing=True,
    )

    # --- Diagnostics UI (left of buttons) ---
    diag_cols = st.columns(3)
    with diag_cols[0]:
        st.caption("**Permission**")
        st.write("Allowed (if you saw a prompt & accepted)")
    with diag_cols[1]:
        st.caption("**Playing**")
        st.write(getattr(getattr(ctx, "state", None), "playing", None))
    with diag_cols[2]:
        st.caption("**ICE state**")
        st.write(getattr(getattr(ctx, "state", None), "iceConnectionState", None))

    # If no frames after 5s → offer fallbacks
    last = getattr(getattr(ctx, "video_processor", None), "last_frame_at", 0.0)
    if (not last) or (time.time() - last > 5):
        with st.expander("Having trouble starting the camera?"):
            st.markdown(
                "- Make sure the tab has **Camera: Allow** (lock icon in address bar).\n"
                "- Close apps that may hold the camera (Teams/Zoom/Phone Link/OBS).\n"
                "- On Windows: Settings → Privacy & security → Camera → Allow for desktop apps.\n"
                "- Try **Fallback widget** (uses Streamlit’s basic capture)."
            )
            if st.button("Use fallback camera widget"):
                ss.fallback_basic_widget = True
                st.rerun()

    # --- Promote capture to pending; show preview & buttons ---
    if ctx and ctx.video_processor and ctx.video_processor.captured is not None and ss.pending_photo_bytes is None:
        import cv2 as _cv
        ok, buf = _cv.imencode(".jpg", ctx.video_processor.captured)
        if ok:
            ss.pending_photo_bytes = buf.tobytes()

    c1, c2 = st.columns(2)
    with c1:
        cont_clicked = st.button("Continue", disabled=(ss.pending_photo_bytes is None))
    with c2:
        retake_clicked = st.button("Retake")

    if ss.pending_photo_bytes is not None:
        st.success("✅ Photo captured! Review below.")
        st.image(ss.pending_photo_bytes, caption="Aligned capture")

    if cont_clicked and ss.pending_photo_bytes is not None:
        ss.camera_photo_bytes = ss.pending_photo_bytes
        ss.photo_captured = True
        st.rerun()

    if retake_clicked:
        if ctx and ctx.video_processor:
            ctx.video_processor.freeze = False
            ctx.video_processor.captured = None
            ctx.video_processor.frames_ok = 0
        ss.pending_photo_bytes = None
        ss.cam_key += 1
        st.rerun()
