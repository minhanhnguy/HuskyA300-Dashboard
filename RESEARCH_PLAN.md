# Phantom Missions: A Full-Stack Visualization Spoofing Framework for ROS 2

## Project Overview
**HuskyA300-Dashboard** is a research platform designed to demonstrate and analyze **Visual Deception Attacks** on autonomous robotic systems. Its primary purpose is to serve as a **Man-In-The-Middle (MITM) testbed**.

It investigates "Human-on-the-Loop" vulnerabilities by decoupling the robot's **Physical State** (Reality) from its **Displayed State** (Dashboard), allowing researchers to inject coherent "Phantom Missions" that spoof the entire ROS 2 Navigation Stack.

## Key Features
- **Live Mode**: Real-time visualization of the robot's pose, path, map (SLAM), and sensor data (LiDAR) directly from a running ROS 2 system.
- **Bag Mode**: Playback and random-access inspection of ROS 2 bag files (MCAP format). Includes timeline scrubbing, "time-travel" map visualization, and command velocity analysis.
- **Hybrid Architecture**: Decoupled backend (Python/FastAPI/ROS 2) and frontend (TypeScript/Vite/Canvas) for performance and flexibility.
- **Shadow State Engine**: A research module capable of intercepting live telemetry and substituting it with a "Golden Run" recording. It maintains semantic consistency across Localization, Perception, and Planning topics to test operator vigilance.

## Technology Stack
- **Backend**:
    - **Language**: Python 3.12+
    - **Framework**: FastAPI (REST API + WebSockets)
    - **ROS 2 Interface**: `rclpy` (ROS 2 Jazzy/Humble compatible)
    - **Data Processing**: NumPy, `rosbag2_py`
- **Frontend**:
    - **Language**: TypeScript
    - **Build Tool**: Vite
    - **Rendering**: HTML5 Canvas (custom rendering engine), D3.js (for zoom/pan behaviors)
    - **Styling**: CSS (Tailwind-like utility classes or custom CSS)

## File Structure & Descriptions

### Root Directory
- **`README.md`**: Setup and usage instructions, including ROS 2 environment sourcing and bag recording commands.
- **`inspect_bag.py`**: A standalone utility script to quickly inspect the topics and types within a bag file.

### Backend (`/backend`)
The backend handles ROS 2 communication, data aggregation, and serves the API.

- **`app.py`**: The entry point for the FastAPI application. Defines REST endpoints (e.g., `/api/v1/status`, `/api/v1/bag/play`) and WebSocket routes (`/ws/stream`, `/ws/map`). Handles the lifecycle of the ROS worker thread.
- **`ros_worker.py`**: Contains the `CmdVelNode` class, which is a ROS 2 node. It subscribes to topics like `/odom`, `/map`, `/scan`, and `/cmd_vel`. It updates the shared state object. Crucially, it includes the Gatekeeper Logic to sever live topic connections when the Attack Mode is triggered.
- **`bag_replay.py`**: Implements the logic for opening, indexing, and querying ROS 2 bag files. It builds time-indexed lookups for maps, poses, and sensor data to support efficient random access (scrubbing).
- **`research_core.py`**: The Shadow State Engine. It manages the PhantomMission logic, handling the synchronization and streaming of 'Golden Run' data to the frontend during an attack simulation.
- **`state.py`**: Defines the `SharedState` class, which acts as a thread-safe bridge between the ROS worker thread (producer) and the FastAPI server (consumer).
- **`models.py`**: Pydantic models defining the structure of API responses (e.g., `PoseHistoryResponse`, `MapFullResponse`).
- **`config.py`**: Configuration constants (topic names, update rates, etc.).
- **`services/`**: Directory containing service layer logic (e.g., `map_service.py`, `pose_service.py`) to keep `app.py` clean.

### Frontend (`/frontend/src`)
The frontend is a single-page application (SPA) that visualizes the data.

- **`main.ts`**: The core application logic. It handles:
    - WebSocket connections to the backend.
    - Canvas rendering loop (drawing the map, robot, path, lidar).
    - User input handling (pan/zoom with D3, timeline slider).
    - State management for switching between Live and Bag modes.
- **`types.ts`**: TypeScript interface definitions that mirror the backend's Pydantic models, ensuring type safety across the stack.
- **`style.css`**: Global CSS styles for the dashboard layout and UI elements.

### Data Directories
- **`bag_files/`**: Directory where `.mcap` bag files should be placed for playback.

## Research Context: The "Full-Stack" Phantom Mission
Unlike simple sensor replay attacks, this framework demonstrates a systemic breach of the autonomous navigation loop. To successfully deceive a supervisor, the system acts as a Digital Twin that falsifies the entire decision-making stack:

### The 6-Layer Spoofing Strategy
To maintain the illusion of a healthy autonomous robot, the dashboard intercepts and falsifies these specific ROS 2 inputs:

1.  **LiDAR (Perception Input):** Replays recorded raw measurements (`/scan`) to match the fake environment geometry.
2.  **Costmap (Representation Input):** Injects a clean occupancy grid (`/global_costmap/costmap`) to convince the operator the path is obstacle-free.
3.  **Map (Internal Memory):** Maintains a consistent static map layer.
4.  **TF Tree (Localization):** Spoofs the `odom` -> `base_link` transform to show smooth movement along the fake path.
5.  **cmd_vel (Actuation):** Displays active velocity commands, making it appear the motors are working to follow the plan.
6.  **Nav2 Path (Planning):** Projects a valid "Green Line" path (`/plan`) to the goal, confirming to the human that the autonomous software is functioning correctly.

### The Attack Workflow
### 1. Generating the "Golden Run" Dataset
To create a valid "Phantom Mission" for injection, record a successful navigation run using the following command to capture the full semantic stack:

```bash
ros2 bag record -o golden_run_01 \
    /tf /tf_static \
    /odom \
    /scan \
    /global_costmap/costmap \
    /plan \
    /cmd_vel \
    /robot_description
```
2.  **Interception:** The backend severs the link to the live robot.
3.  **Injection:** The `ShadowState` engine streams the Golden Run data to the dashboard.
4.  **Result:** The operator sees the robot succeeding, while the real robot is idle, stuck, or hijacked.

## ⚡ Installation & Usage

### Prerequisites
- ROS 2 (Jazzy or Humble)
- Python 3.12+
- Node.js 18+

### 1. Setup Backend
```bash
cd backend
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
# Ensure ROS 2 is sourced (e.g., source /opt/ros/jazzy/setup.bash)
```

### 2. Setup Frontend
```bash
cd frontend
npm install
npm run build
```

### 3. Running the Attack Simulation
**1. Start the Backend:**
```bash
# Serves API at http://localhost:8000
source /opt/ros/jazzy/setup.bash
python -m uvicorn backend.app:app --host 0.0.0.0 --port 8000
```

**2. Start the Frontend:**
```bash
# Serves UI at http://localhost:5173
npm run dev
```

**3. Trigger the Injection:**
1. Open the Dashboard in your browser.
2. Navigate to the Debug/Research panel.
3. Upload your `golden_run_01.mcap` file.
4. Click "Inject Shadow State" to sever the live link and begin the spoofing sequence.

## ⚠️ Ethical Disclaimer & Security Notice
This software is provided for **educational and research purposes only**. It is intended to demonstrate vulnerabilities in "Human-on-the-Loop" robotic systems to improve future security standards.
- **Do not use this software on systems you do not own or have explicit permission to test.**
- The authors are not responsible for any damage caused by the misuse of this software.