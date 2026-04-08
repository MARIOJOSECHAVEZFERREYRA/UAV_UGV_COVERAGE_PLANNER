import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  test: {
    environment: 'node',
  },
  server: {
    port: 5173,
    proxy: {
      '/mission': 'http://localhost:8000',
      '/fields':  'http://localhost:8000',
      '/drones':  'http://localhost:8000',
      '/health':  'http://localhost:8000',
    },
  },
})
