import './assets/main.css'

import { createApp } from 'vue'
import App from './App.vue'
import router from './router'

// Bootstrap CSS
import "bootstrap/dist/css/bootstrap.min.css";
import "bootstrap-icons/font/bootstrap-icons.css";


// Optional: Bootstrap JS (if you need modals, dropdowns, etc.)
import "bootstrap";


const app = createApp(App)

app.use(router)

app.mount('#app')
