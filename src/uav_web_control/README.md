# uav_web_control

Small starter Flask web app to send simple "fly" commands to UAVs.

Usage

1. Create a Python virtualenv and install dependencies:

```bash
cd src/uav_web_control
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

2. Run the app:

```bash
# option A (flask)
export FLASK_APP=app.py
flask run --host=0.0.0.0

# option B (direct)
python app.py
```

3. Start rosbridge websocket (default port 9090):

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

4. Open `http://localhost:5000` and click a UAV's "Fly" button.
