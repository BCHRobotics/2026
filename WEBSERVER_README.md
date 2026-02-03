# RoboRIO Vision Web Server

## Overview

The RoboRIO Vision Web Server provides a browser-based dashboard for monitoring PhotonVision cameras, AprilTag detection, object tracking, and pose estimation in real-time. This runs directly on the RoboRIO and is automatically started when the robot code initializes.

## Features

- **Real-time Monitoring**: Auto-refreshes every 500ms
- **PhotonVision Integration**: Displays camera status, latency, FPS, and target counts
- **AprilTag Detection**: Shows detected tags, distances, and angles
- **Object Detection**: Monitors ball/object detection with yaw, pitch, and area
- **Pose Estimation**: Displays vision-based pose estimates (X, Y, rotation, timestamp)
- **Beautiful UI**: Modern gradient design with glass-morphism cards
- **REST API**: JSON endpoints for programmatic access

## Access

### Local Network (Recommended)
- **IP Address**: `http://10.TE.AM.2:8082`
  - Replace `TE.AM` with your team number (e.g., team 2026 → `http://10.20.26.2:8082`)
- **mDNS**: `http://roborio-TEAM-frc.local:8082`
  - Replace `TEAM` with your team number (e.g., `http://roborio-2026-frc.local:8082`)

### Driver Station
Access from any device connected to the robot network:
- Tablets (for pit crew)
- Driver station laptop
- Field monitors
- Any smartphone/device on the robot WiFi

## Endpoints

### `/` - Main Dashboard
Interactive web interface with auto-updating data visualization.

### `/api/vision` - Vision Data (JSON)
Returns all vision data including cameras, AprilTags, objects, and pose estimation.

**Example Response:**
```json
{
  "cameras": [
    {
      "name": "Camera_0",
      "connected": true,
      "latency": 35.2,
      "fps": 30,
      "hasTargets": true,
      "targetCount": 2
    },
    {
      "name": "banana_1",
      "connected": true,
      "latency": 28.5,
      "fps": 30,
      "hasTargets": false,
      "targetCount": 0
    }
  ],
  "apriltag": {
    "count": 2,
    "bestId": 5,
    "distance": 2.45,
    "yaw": -12.3
  },
  "object": {
    "visible": true,
    "yaw": 5.2,
    "pitch": -3.1,
    "area": 12.5
  },
  "pose": {
    "hasVisionPose": true,
    "x": 3.45,
    "y": 2.67,
    "rotation": 45.2,
    "timestamp": 12345.678
  }
}
```

### `/api/status` - Server Status (JSON)
Returns the web server status and port number.

**Example Response:**
```json
{
  "running": true,
  "port": 8082
}
```

## SmartDashboard Integration

The web server publishes its status to SmartDashboard:
- `WebServer/Running`: Boolean indicating if server is active
- `WebServer/Port`: Port number (8082)

## Architecture

### Java Implementation
- **Location**: `src/main/java/frc/robot/webserver/VisionWebServer.java`
- **HTTP Server**: Uses `com.sun.net.httpserver.HttpServer` (built into Java)
- **Threading**: Runs on a fixed thread pool (2 threads)
- **Data Source**: Reads from SmartDashboard entries published by Vision subsystem

### Auto-Start
The web server is automatically started in `RobotContainer.java` constructor:
```java
private final VisionWebServer m_webServer = new VisionWebServer(m_vision);

public RobotContainer() {
    // ...
    m_webServer.start();
    // ...
}
```

## Troubleshooting

### Cannot Connect to Web Server

1. **Check RoboRIO Connection**
   - Ensure you're connected to the robot network
   - Verify RoboRIO IP address with Driver Station
   - Ping the RoboRIO: `ping 10.TE.AM.2`

2. **Check Robot Code is Running**
   - Verify robot code is deployed and enabled
   - Check SmartDashboard for `WebServer/Running = true`

3. **Check Port 8082**
   - Ensure no firewall is blocking port 8082
   - Try accessing the status endpoint: `http://10.TE.AM.2:8082/api/status`

### No Data Displayed

1. **Check PhotonVision**
   - Ensure PhotonVision is running on coprocessor
   - Verify camera configuration in VisionConstants.java
   - Check NetworkTables for vision data

2. **Check Vision Subsystem**
   - Verify Vision subsystem is initialized in RobotContainer
   - Check SmartDashboard for vision entries (Vision/Camera_0/*, Vision/AprilTag/*, etc.)

3. **Browser Cache**
   - Hard refresh your browser (Ctrl+F5 or Cmd+Shift+R)
   - Clear browser cache
   - Try incognito/private mode

### Web Server Won't Start

Check console output for error messages. Common issues:
- Port 8082 already in use (change PORT constant if needed)
- Network interface not available
- Java security restrictions

## Development

### Modifying the Dashboard
Edit `VisionWebServer.java`, specifically the `generateDashboardHTML()` method. The HTML, CSS, and JavaScript are embedded as a Java text block for easy deployment.

### Adding New Endpoints
Create a new handler class implementing `HttpHandler` and register it in the `start()` method:
```java
server.createContext("/api/custom", new CustomHandler());
```

### Changing the Port
Modify the `PORT` constant in `VisionWebServer.java`:
```java
private static final int PORT = 8082;  // Change to desired port
```

### Customizing Data Sources
Update the `generateVisionJSON()` method to read additional SmartDashboard entries or directly query subsystems.

## Performance Notes

- **Update Rate**: Dashboard refreshes every 500ms (configurable in JavaScript)
- **Thread Pool**: 2 threads handle HTTP requests (sufficient for multiple clients)
- **Memory**: Minimal overhead, all data comes from SmartDashboard
- **CPU**: Negligible impact on robot control loop

## Security

⚠️ **Warning**: This web server has NO authentication or encryption. It's designed for use on the isolated robot network only. Do not expose it to public networks.

## Credits

Built for FRC Team 2026 using:
- WPILib 2026 framework
- PhotonVision for computer vision
- Java's built-in HTTP server

---

**Questions?** Check the code comments in `VisionWebServer.java` or contact the programming team.
