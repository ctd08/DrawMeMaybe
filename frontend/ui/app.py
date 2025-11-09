import streamlit as st
from ui_screensaver import show_screensaver
from ui_consent import show_consent_form
from ui_camera import show_camera_stage
from ui_chat import show_chat_ui
from utils.styles import inject_global_css

st.set_page_config(page_title="Rob Ross Chat", page_icon="🎨", layout="wide")

# Inject your CSS once
inject_global_css()

# ----------------------------
# URL param: ?touched=1 → move to consent
# ----------------------------
try:
    # Newer Streamlit API
    qp = st.query_params
    if "touched" in qp and qp["touched"] == "1":
        st.session_state.touched = True
        st.session_state.route = "consent"  # <-- ensure we route correctly
        st.query_params.clear()             # remove param
        st.rerun()
except Exception:
    # Fallback for older versions
    qp = st.experimental_get_query_params()
    if qp.get("touched", ["0"])[0] == "1":
        st.session_state.touched = True
        st.session_state.route = "consent"  # <-- ensure we route correctly
        st.experimental_set_query_params()  # clear params
        st.rerun()

# ----------------------------
# State bootstrap
# ----------------------------
if "touched" not in st.session_state:
    st.session_state.touched = False  # set True to skip screensaver while dev
if "consent_accepted" not in st.session_state:
    st.session_state.consent_accepted = False
if "photo_captured" not in st.session_state:
    st.session_state.photo_captured = False
if "route" not in st.session_state:
    st.session_state.route = None  # prefer explicit route when set by pages

# ----------------------------
# Simple router
# ----------------------------
def derive_route() -> str:
    """Prefer explicit route; otherwise infer from flow flags."""
    if st.session_state.route:
        return st.session_state.route
    if not st.session_state.touched:
        return "screensaver"
    if not st.session_state.consent_accepted:
        return "consent"
    if not st.session_state.photo_captured:
        return "camera"
    return "chat"

route = derive_route()

# ----------------------------
# Render according to route
# ----------------------------
if route == "screensaver":
    show_screensaver()

elif route == "consent":
    # show_consent_form will set:
    #   st.session_state.consent_accepted = True
    #   st.session_state.route = "camera"
    # and call st.rerun() on Accept
    show_consent_form()

elif route == "camera":
    # Your camera stage should set:
    #   st.session_state.photo_captured = True
    #   st.session_state.route = "chat"   (when ready)
    show_camera_stage()

elif route == "chat":
    show_chat_ui()

else:
    # Fallback: derive again
    st.session_state.route = derive_route()
    st.rerun()
