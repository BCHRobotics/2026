# Robot Utilities

Python utilities for monitoring and debugging robot systems.

## NetworkTables Vision Monitor

Real-time monitoring of vision, odometry, and camera data from the robot.

### Quick Start (Recommended)

Use the launcher scripts that automatically set up the environment:

**Windows (PowerShell):**
```powershell
.\run_vision_monitor.ps1 --team 2026
```

**Windows (Command Prompt):**
```cmd
run_vision_monitor.bat --team 2026
```

**Linux/Mac (Bash):**
```bash
./run_vision_monitor.sh --team 2026
```

The launcher scripts will:
- ✅ Create a Python virtual environment (first run only)
- ✅ Install all dependencies automatically
- ✅ Activate the environment
- ✅ Launch the vision monitor

### Manual Installation (Optional)

If you prefer to run manually:

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows PowerShell)
.\venv\Scripts\Activate.ps1

# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the monitor
python nt_vision_monitor.py --team 2026
```

### Usage Examples

**Connect to robot via team number:**
```bash
./run_vision_monitor.sh --team 2026
```

**Connect to robot via IP address:**
```bash
./run_vision_monitor.sh --ip 10.20.26.2
```

**Connect to robot simulator (localhost):**
```bash
./run_vision_monitor.sh --simulator
```

**Custom refresh rate:**
```bash
./run_vision_monitor.sh --team 2026 --rate 0.5
```

### Features

- **Real-time display** of vision data with auto-refresh
- **Built-in web interface** on port 8081 for remote monitoring
- **AprilTag detection** - Shows detected tag IDs, distances, and angles
- **Ball/Object detection** - Displays yaw, pitch, and area
- **Pose estimation** - Shows robot position from vision
- **Odometry data** - Current pose, velocity, and heading
- **PhotonVision cameras** - Status for all configured cameras
- **SmartDashboard integration** - Displays all vision-related values

### Web Interface

The monitor automatically starts a web server on **port 8081** that provides a beautiful, auto-updating dashboard:

**Access the web interface:**
- Open your browser to: `http://localhost:8081`
- Or from another computer: `http://[COMPUTER_IP]:8081`

**Features:**
- 📊 Real-time data updates every 500ms
- 🎨 Modern, gradient UI with glass-morphism design
- 📱 Responsive layout for mobile/tablet viewing
- 🔄 Automatic reconnection handling
- 📸 Dedicated PhotonVision camera diagnostics

**Disable web server (terminal only):**
```bash
./run_vision_monitor.sh --team 2026 --no-web
```

### Display Sections

1. **Vision Data**
   - AprilTag detection (count, ID, distance, yaw)
   - Ball detection (visibility, yaw, pitch, area)
   - Pose estimation (estimated pose, timestamp)
   - Camera status (enabled cameras, latency)

2. **Odometry Data**
   - Robot pose (X, Y, rotation)
   - Velocity (X, Y components)
   - Heading

3. **PhotonVision Cameras**
   - Per-camera status (Camera_0, banana_1)
   - Target detection
   - Latency measurements
   - Target angle and area data

4. **SmartDashboard Values**
   - All vision-related keys published to SmartDashboard

### Exit

Press `Ctrl+C` to exit the monitor.

## Troubleshooting

**Connection timeout:**
- Verify robot is powered on
- Check network connection (WiFi or USB)
- Confirm team number or IP address is correct
- For simulator: ensure robot code is running in simulator mode

**No data displayed:**
- Verify robot code is publishing to NetworkTables
- Check that Vision subsystem is initialized
- Ensure cameras are properly configured and enabled

**Missing data:**
- Some fields only appear when data is available
- AprilTag data only shows when tags are detected
- Ball data only shows when balls are visible
- PhotonVision data requires cameras to be connected and configured
