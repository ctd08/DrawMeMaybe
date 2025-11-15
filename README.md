# DrawMeMaybe – Web Application

This branch contains the rewritten version of the DrawMeMaybe app using a proper Frontend/Backend architecture, replacing the previous Streamlit-only prototype.

The application is a touch-friendly kiosk experience where a user:

- Views a screensaver
- Accepts the consent form
- Aligns their face inside a contour
- Captures a photo
- Inputs a hobby or interest
- Receives a generated caption for their caricature

🖥️ Frontend Setup

Go into the frontend folder:
  cd frontend

Install dependencies:
npm install


Run the development server:
npm run dev


You will see output similar to:
➜  Local:   http://localhost:5173/

Open that link in a browser.


📱 Test on a tablet / phone

Run Vite with host enabled:
npm run dev -- --host


Find your computer’s local IP (example: 192.168.1.20)

On the tablet browser:
http://192.168.1.20:5173/


⚠️ Camera access requires HTTPS on mobile Safari. For development, localhost works without HTTPS, but LAN IPs may trigger permission issues on iOS. Chrome on Android has fewer restrictions.
