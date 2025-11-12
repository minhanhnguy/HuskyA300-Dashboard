# HuskyA300-Dashboard

Source the Ros2 Environment with:

```bash
source /opt/ros/jazzy/setup.bash
```

Run the backend with:

```bash
python -m uvicorn backend.app:app --host 0.0.0.0 --port 8000 --reload --workers 1
```

Open another terminal, run the frontend with:

```bash
npm run dev
```