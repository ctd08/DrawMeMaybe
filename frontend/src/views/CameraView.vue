<template>
  <div class="app-screen">
    <div class="app-card">
      <!-- Title / instructions -->
      <h1 class="app-card-title mb-1">
        <i class="bi bi-camera me-2"></i>
        Position yourself in the frame
      </h1>
      <p class="app-card-subtitle">
        Laptop → use the webcam. Phone/Tablet → use the front camera.
        Make sure you allow camera access. Works best on HTTPS or localhost.
      </p>

      <!-- Camera preview with SVG head contour -->
      <div class="camera-preview mb-3">
        <div class="camera-frame-wrapper">
          <video
            ref="videoRef"
            class="camera-video"
            autoplay
            playsinline
          ></video>

          <!-- SVG head contour overlay -->
          <div class="head-overlay">
            <svg
              class="head-svg"
              viewBox="0 0 200 260"
              xmlns="http://www.w3.org/2000/svg"
            >
              <!-- Head -->
              <path
                d="M100 10
                   C 65 10, 40 38, 40 80
                   C 40 120, 55 150, 70 165
                   C 66 185, 58 205, 55 225
                   L 145 225
                   C 142 205, 134 185, 130 165
                   C 145 150, 160 120, 160 80
                   C 160 38, 135 10, 100 10 Z"
                fill="none"
                stroke="rgba(255,255,255,0.95)"
                stroke-width="4"
              />
              <!-- Shoulders -->
              <path
                d="M55 225
                   C 35 235, 20 250, 15 260
                   L 185 260
                   C 180 250, 165 235, 145 225 Z"
                fill="none"
                stroke="rgba(255,255,255,0.85)"
                stroke-width="4"
              />
            </svg>
          </div>

          <!-- Optional vignette -->
          <div class="face-mask"></div>
        </div>
      </div>

      <!-- Info / error -->
      <div v-if="errorMessage" class="text-danger mb-2">
        {{ errorMessage }}
      </div>
      <div v-else class="text-muted small mb-2">
        Align your face inside the contour, then tap <strong>Capture</strong>.
      </div>

      <!-- Hidden canvas used only for grabbing a frame -->
      <canvas ref="canvasRef" class="d-none"></canvas>

      <!-- Buttons -->
      <div class="mt-3 d-flex flex-wrap justify-content-end gap-2">
        <button
          type="button"
          class="btn btn-outline-secondary btn-pill"
          @click="onRetake"
          :disabled="!hasPhoto"
        >
          Retake
        </button>

        <button
          type="button"
          class="btn btn-secondary btn-pill"
          @click="capturePhoto"
          :disabled="isStarting"
        >
          {{ hasPhoto ? "Capture again" : "Capture" }}
        </button>

        <button
          type="button"
          class="btn btn-dark btn-pill"
          @click="onContinue"
          :disabled="!hasPhoto"
        >
          Continue
        </button>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted, onBeforeUnmount } from "vue";
import { useRouter } from "vue-router";

const router = useRouter();

const videoRef = ref(null);
const canvasRef = ref(null);

const isStarting = ref(false);
const hasPhoto = ref(false);
const photoDataUrl = ref(null);
const errorMessage = ref("");

const mediaStream = ref(null);

async function startCamera() {
  isStarting.value = true;
  errorMessage.value = "";

  try {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: { facingMode: "user" },
      audio: false,
    });

    mediaStream.value = stream;

    if (videoRef.value) {
      videoRef.value.srcObject = stream;
    }
  } catch (err) {
    console.error("Error accessing camera:", err);
    errorMessage.value =
      "Could not access the camera. Please allow permissions and reload the page.";
  } finally {
    isStarting.value = false;
  }
}

function stopCamera() {
  if (mediaStream.value) {
    mediaStream.value.getTracks().forEach((track) => track.stop());
    mediaStream.value = null;
  }
}

function capturePhoto() {
  const video = videoRef.value;
  const canvas = canvasRef.value;
  if (!video || !canvas) return;

  const w = video.videoWidth;
  const h = video.videoHeight;

  if (!w || !h) {
    errorMessage.value =
      "Camera not ready yet. Wait a moment and try again.";
    return;
  }

  canvas.width = w;
  canvas.height = h;

  const ctx = canvas.getContext("2d");
  ctx.drawImage(video, 0, 0, w, h);

  const dataUrl = canvas.toDataURL("image/jpeg", 0.9);
  photoDataUrl.value = dataUrl;
  hasPhoto.value = true;

  console.log("Captured photo data URL length:", dataUrl.length);
}

function onRetake() {
  hasPhoto.value = false;
  photoDataUrl.value = null;
  errorMessage.value = "";
}

function onContinue() {
  if (!hasPhoto.value) return;

  // TODO later: send photoDataUrl.value to backend before routing
  router.push("/chat");
}

onMounted(() => {
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    errorMessage.value =
      "This browser does not support camera access. Please try a different device or browser.";
    return;
  }
  startCamera();
});

onBeforeUnmount(() => {
  stopCamera();
});
</script>

<style scoped>
.camera-preview {
  background: #000;
  border-radius: 18px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  aspect-ratio: 4 / 3;
}

.camera-frame-wrapper {
  position: relative;
  width: 100%;
  height: 100%;
}

.camera-video {
  width: 100%;
  height: 100%;
  object-fit: cover;
}

/* Center and scale the head overlay */
.head-overlay {
  position: absolute;
  inset: 50%;
  transform: translate(-50%, -50%);
  width: 30%;
  height: 68%;
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 2;
}

.head-svg {
  width: 100%;
  height: 100%;
}

/* Soft vignette outside the face area */
.face-mask {
  position: absolute;
  inset: 0;
  background:
    radial-gradient(
      ellipse at center,
      rgba(0, 0, 0, 0) 28%,
      rgba(0, 0, 0, 0.55) 80%
    );
  pointer-events: none;
  z-index: 1;
}
</style>
