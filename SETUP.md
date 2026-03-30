# Setup y Ejecución - UAV-UAG Planner

Documentación completa para configurar y ejecutar el proyecto.

## Requisitos previos

- Node.js 18+ (para el frontend)
- Python 3.9+ (para el backend)
- npm o yarn (gestor de paquetes para Node)
- Git (para clonar/trabajar en el repositorio)

## Ejecución rápida

Si ya está todo configurado:

### Terminal 1 - Backend
```bash
source venv/bin/activate
cd backend
uvicorn main:app --reload
```

### Terminal 2 - Frontend
```bash
cd frontend
npm run dev
```

---

## Instalación completa desde cero

### 1. Backend (Python + FastAPI)

#### a) Crear y activar entorno virtual
```bash
# Crear entorno virtual (si no existe)
python3 -m venv venv

# Activar entorno virtual
# En Linux/Mac:
source venv/bin/activate

# En Windows:
venv\Scripts\activate
```

#### b) Instalar dependencias
```bash
pip install -r backend/requirements.txt
```

**Dependencias principales:**
- fastapi>=0.111.0 - Framework web
- uvicorn[standard]>=0.29.0 - Servidor ASGI
- sqlalchemy>=2.0.0 - ORM para base de datos
- pydantic>=2.6.0 - Validación de datos

#### c) Ejecutar servidor
```bash
cd backend
uvicorn main:app --reload
```

El servidor estará en: http://localhost:8000

- Documentación API: http://localhost:8000/docs (Swagger UI)
- Health check: http://localhost:8000/health

---

### 2. Frontend (React + Vite)

#### a) Instalar dependencias
```bash
cd frontend
npm install
```

**Dependencias principales:**
- react@18.3.1 - Framework UI
- react-dom@18.3.1 - Renderización
- maplibre-gl@4.3.2 - Mapas interactivos
- vite@5.3.4 - Build tool y dev server

#### b) Ejecutar servidor de desarrollo
```bash
npm run dev
```

El servidor estará en: http://localhost:5173

#### c) Build para producción
```bash
npm run build
```

Genera los archivos en frontend/dist/

#### d) Preview del build
```bash
npm run preview
```

---

## Estructura del proyecto

```
UAV-UAG_mission_planner/
├── backend/                 # API FastAPI (Python)
│   ├── main.py             # Punto de entrada
│   ├── database.py         # Configuración BD
│   ├── requirements.txt    # Dependencias Python
│   └── routers/            # Endpoints API
├── frontend/               # Aplicación React (JavaScript)
│   ├── src/
│   │   ├── components/     # Componentes React
│   │   ├── App.jsx
│   │   └── main.jsx
│   ├── package.json        # Dependencias Node
│   └── vite.config.js      # Configuración Vite
├── desktop/                # Aplicación de escritorio
├── tests/                  # Tests del proyecto
├── venv/                   # Entorno virtual Python
└── SETUP.md               # Este archivo
```

---

## Comunicación Frontend - Backend

El frontend está configurado para comunicarse con el backend en:
- Backend: http://localhost:8000
- Frontend: http://localhost:5173

CORS configurado: El backend permite requests desde http://localhost:5173

---

## Tests

### Tests Python
```bash
source venv/bin/activate
pytest tests/
```

### Tests JavaScript
```bash
cd frontend
npm test  # (si está configurado)
```

---

## Solución de problemas

### Puerto 8000 en uso
```bash
# Encuentra qué está usando el puerto 8000
lsof -i :8000

# O ejecuta el servidor en otro puerto
uvicorn main:app --reload --port 8001
```

### Puerto 5173 en uso
```bash
# El servidor Vite automáticamente usa 5174 si 5173 está ocupado
npm run dev
```

### Entorno virtual no se activa
```bash
# En Mac/Linux, asegúrate de estar en la raíz del proyecto
source ./venv/bin/activate

# En Windows
.\venv\Scripts\activate
```

### Dependencias desactualizadas
```bash
# Backend
pip install --upgrade -r backend/requirements.txt

# Frontend
npm update
```

---

## APIs principales

Disponibles en http://localhost:8000/docs (Swagger UI)

### Routers incluidos:
- fields - Gestión de campos agrícolas
- mission - Gestión de misiones
- telemetry - Datos de telemetría
- drones - Gestión de drones

---

## Flujo de desarrollo recomendado

1. Terminal 1: Inicia el backend
   ```bash
   source venv/bin/activate
   cd backend && uvicorn main:app --reload
   ```

2. Terminal 2: Inicia el frontend
   ```bash
   cd frontend && npm run dev
   ```

3. Terminal 3: (Opcional) Ejecuta tests o comandos git
   ```bash
   git status
   pytest tests/
   ```

4. Abre http://localhost:5173 en tu navegador

---

## Variables de entorno (si es necesario)

Crear archivo .env en la raíz o en backend/:

```bash
# Backend
DATABASE_URL=sqlite:///./test.db
DEBUG=True

# Frontend (si se requiere)
VITE_API_URL=http://localhost:8000
```

---

## Despliegue

Ver documentación adicional en la carpeta /docs o contacta al equipo de desarrollo.

---

## Contacto y soporte

Para problemas o preguntas, abre un issue en el repositorio.
