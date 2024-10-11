// import './assets/main.css'

import { createApp } from 'vue';
import PrimeVue from 'primevue/config';
import App from './App.vue'
import Button from 'primevue/button';

const app = createApp(App)
app.use(PrimeVue)
app.component('Button', Button)
app.mount('#app')

// const { app, BrowserWindow } = require('electron')

// const createWindow = () => {
//     const win = new BrowserWindow({
//       width: 800,
//       height: 600
//     })
  
//     win.loadFile('index.html')
//   }

//   app.whenReady().then(() => {
//     createWindow()
//   })