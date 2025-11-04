import streamlit as st

def show_screensaver():
    st.markdown("""
    <style>
      .screensaver {
        position: fixed; inset: 0;
        display: flex; flex-direction: column;
        align-items: center; justify-content: center;
        background: #f5e3c3;
        z-index: 9999;
        -webkit-user-select: none; user-select: none;
        overflow: hidden;
        opacity: 1;
        transform: translateY(0);
        transition: opacity .35s ease, transform .35s ease;
      }
      .screensaver.fade-out {
        opacity: 0;
        transform: translateY(-8px);
      }
      .logo {
        width: min(60vw, 380px);
        animation: float 3s ease-in-out infinite;
      }
      .hint { margin-top: 1rem; color: #333; font-weight: 600; }
      @keyframes float {
        0% { transform: translateY(0); }
        50% { transform: translateY(-12px); }
        100% { transform: translateY(0); }
      }
      .overlay {
        position: fixed; inset: 0;
        background: transparent;
        cursor: pointer;
        z-index: 10000;
      }
    </style>

    <div class="screensaver" id="screensaver-root">
      <img class="logo"
           src="https://raw.githubusercontent.com/Cristina2000-hub/DrawMeMaybe/frontend/frontend/uploads/Designer%20(1).png"
           alt="logo" />
      <div class="hint">👆 Tap anywhere to start</div>
      <div id="tap-overlay" class="overlay" title="Start"></div>
    </div>
    """, unsafe_allow_html=True)

    # JS: add fade-out class, then navigate to ?touched=1 after 350ms
    st.markdown("""
    <script>
      (function () {
        const overlay = document.getElementById("tap-overlay");
        const root = document.getElementById("screensaver-root");
        if (!overlay || !root) return;

        function go() {
          try {
            root.classList.add("fade-out");
            setTimeout(() => {
              const url = new URL(window.location.href);
              url.searchParams.set("touched", "1");
              window.location.href = url.toString();
            }, 350); // match CSS transition
          } catch (e) {
            root.classList.add("fade-out");
            setTimeout(() => {
              const sep = window.location.search ? "&" : "?";
              window.location.href = window.location.href + sep + "touched=1";
            }, 350);
          }
        }
        overlay.addEventListener("click", go, { passive: true });
        overlay.addEventListener("touchstart", go, { passive: true });
      })();
    </script>
    """, unsafe_allow_html=True)

    # Optional fallback button if JS is blocked
    if st.button("Start 🎨", use_container_width=True):
        st.session_state.touched = True
        st.rerun()
