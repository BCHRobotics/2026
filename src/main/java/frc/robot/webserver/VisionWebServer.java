package frc.robot.webserver;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.Executors;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Vision;

/**
 * Web server for vision diagnostics on RoboRIO.
 * 
 * Provides a web interface on port 8082 for monitoring:
 * - PhotonVision camera status
 * - AprilTag detection
 * - Ball/object detection
 * - Pose estimation
 * - Camera latency and performance
 * 
 * Access: http://10.TE.AM.2:8082 or http://roborio-TEAM-frc.local:8082
 */
public class VisionWebServer {
    
    private static final int PORT = 8082;
    private HttpServer server;
    @SuppressWarnings("unused")
    private Vision visionSubsystem;
    private boolean running = false;
    
    /**
     * Create a new vision web server.
     * 
     * @param vision The vision subsystem to monitor
     */
    public VisionWebServer(Vision vision) {
        this.visionSubsystem = vision;
    }
    
    /**
     * Start the web server.
     * 
     * @return true if started successfully, false otherwise
     */
    public boolean start() {
        try {
            server = HttpServer.create(new InetSocketAddress(PORT), 0);
            server.createContext("/", new DashboardHandler());
            server.createContext("/api/vision", new VisionDataHandler());
            server.createContext("/api/status", new StatusHandler());
            server.setExecutor(Executors.newFixedThreadPool(2));
            server.start();
            running = true;
            
            System.out.println("Vision Web Server started on port " + PORT);
            SmartDashboard.putBoolean("WebServer/Running", true);
            SmartDashboard.putNumber("WebServer/Port", PORT);
            
            return true;
        } catch (IOException e) {
            System.err.println("Failed to start web server: " + e.getMessage());
            SmartDashboard.putBoolean("WebServer/Running", false);
            return false;
        }
    }
    
    /**
     * Stop the web server.
     */
    public void stop() {
        if (server != null) {
            server.stop(0);
            running = false;
            SmartDashboard.putBoolean("WebServer/Running", false);
            System.out.println("Web server stopped");
        }
    }
    
    /**
     * Check if the web server is running.
     * 
     * @return true if running
     */
    public boolean isRunning() {
        return running;
    }
    
    /**
     * Handler for the main dashboard page.
     */
    private class DashboardHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            String html = generateDashboardHTML();
            byte[] response = html.getBytes(StandardCharsets.UTF_8);
            
            exchange.getResponseHeaders().set("Content-Type", "text/html; charset=UTF-8");
            exchange.sendResponseHeaders(200, response.length);
            
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(response);
            }
        }
    }
    
    /**
     * Handler for vision data API endpoint.
     */
    private class VisionDataHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            String json = generateVisionJSON();
            byte[] response = json.getBytes(StandardCharsets.UTF_8);
            
            exchange.getResponseHeaders().set("Content-Type", "application/json");
            exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
            exchange.sendResponseHeaders(200, response.length);
            
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(response);
            }
        }
    }
    
    /**
     * Handler for status API endpoint.
     */
    private class StatusHandler implements HttpHandler {
        @Override
        public void handle(HttpExchange exchange) throws IOException {
            String json = generateStatusJSON();
            byte[] response = json.getBytes(StandardCharsets.UTF_8);
            
            exchange.getResponseHeaders().set("Content-Type", "application/json");
            exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
            exchange.sendResponseHeaders(200, response.length);
            
            try (OutputStream os = exchange.getResponseBody()) {
                os.write(response);
            }
        }
    }
    
    /**
     * Generate the main dashboard HTML.
     */
    private String generateDashboardHTML() {
        return """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RoboRIO Vision Diagnostics</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: #fff;
            padding: 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 10px;
            font-size: 2.5em;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .subtitle {
            text-align: center;
            opacity: 0.9;
            margin-bottom: 20px;
            font-size: 1.2em;
        }
        .status-bar {
            display: flex;
            justify-content: space-around;
            margin-bottom: 30px;
            flex-wrap: wrap;
            gap: 15px;
        }
        .status-item {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            padding: 15px 30px;
            border-radius: 10px;
            text-align: center;
            min-width: 150px;
        }
        .status-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .status-value {
            font-size: 1.8em;
            font-weight: bold;
        }
        .card {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 15px;
            padding: 25px;
            margin-bottom: 20px;
            box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
        }
        .card h2 {
            margin-bottom: 20px;
            border-bottom: 2px solid rgba(255,255,255,0.3);
            padding-bottom: 10px;
            font-size: 1.5em;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 20px;
        }
        .data-box {
            background: rgba(0, 0, 0, 0.2);
            padding: 20px;
            border-radius: 10px;
            border-left: 4px solid #4CAF50;
        }
        .data-box.warning { border-left-color: #FF9800; }
        .data-box.error { border-left-color: #F44336; }
        .data-label {
            font-size: 0.95em;
            opacity: 0.8;
            margin-bottom: 8px;
        }
        .data-value {
            font-size: 1.6em;
            font-weight: bold;
            word-break: break-word;
        }
        .camera-section {
            background: rgba(0, 0, 0, 0.3);
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 15px;
        }
        .camera-title {
            font-size: 1.3em;
            margin-bottom: 15px;
            color: #4CAF50;
        }
        .footer {
            text-align: center;
            margin-top: 30px;
            opacity: 0.7;
            font-size: 0.9em;
        }
        .connected { color: #4CAF50; }
        .disconnected { color: #F44336; }
        .loading {
            text-align: center;
            padding: 40px;
            opacity: 0.6;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>RoboRIO Vision Diagnostics</h1>
        <div class="subtitle">Real-time PhotonVision & Camera Monitoring</div>
        
        <div class="status-bar">
            <div class="status-item">
                <div class="status-label">Server Status</div>
                <div class="status-value" id="server-status">Online</div>
            </div>
            <div class="status-item">
                <div class="status-label">Update Rate</div>
                <div class="status-value">500ms</div>
            </div>
            <div class="status-item">
                <div class="status-label">Port</div>
                <div class="status-value">8082</div>
            </div>
        </div>
        
        <div class="card">
            <h2>PhotonVision Cameras</h2>
            <div id="camera-data">
                <div class="loading">Loading camera data...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>AprilTag Detection</h2>
            <div class="grid" id="apriltag-data">
                <div class="loading">Loading AprilTag data...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>Object Detection</h2>
            <div class="grid" id="object-data">
                <div class="loading">Loading object detection data...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>Pose Estimation</h2>
            <div class="grid" id="pose-data">
                <div class="loading">Loading pose data...</div>
            </div>
        </div>
        
        <div class="footer">
            RoboRIO Web Server | Auto-refresh every 500ms | Port 8082
        </div>
    </div>
    
    <script>
        let updateInterval;
        
        function updateData() {
            fetch('/api/vision')
                .then(response => response.json())
                .then(data => {
                    updateCameraData(data.cameras);
                    updateAprilTagData(data.apriltag);
                    updateObjectData(data.object);
                    updatePoseData(data.pose);
                })
                .catch(error => {
                    console.error('Error fetching vision data:', error);
                    document.getElementById('server-status').innerHTML = 'Error';
                });
            
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('server-status').innerHTML = 'Online';
                })
                .catch(error => {
                    document.getElementById('server-status').innerHTML = 'Offline';
                });
        }
        
        function updateCameraData(cameras) {
            const container = document.getElementById('camera-data');
            if (!cameras || cameras.length === 0) {
                container.innerHTML = '<div class="loading">No camera data available</div>';
                return;
            }
            
            let html = '';
            cameras.forEach(cam => {
                html += `
                    <div class="camera-section">
                        <div class="camera-title">${cam.name}</div>
                        <div class="grid">`;
                            <div class="data-box ${cam.connected ? '' : 'error'}">
                                <div class="data-label">Status</div>
                                <div class="data-value">${cam.connected ? 'Connected' : 'Disconnected'}</div>
                            </div>
                            <div class="data-box">
                                <div class="data-label">Latency</div>
                                <div class="data-value">${cam.latency.toFixed(1)} ms</div>
                            </div>
                            <div class="data-box">
                                <div class="data-label">FPS</div>
                                <div class="data-value">${cam.fps}</div>
                            </div>
                            <div class="data-box ${cam.hasTargets ? '' : 'warning'}">
                                <div class="data-label">Targets</div>
                                <div class="data-value">${cam.targetCount}</div>
                            </div>
                        </div>
                    </div>
                `;
            });
            container.innerHTML = html;
        }
        
        function updateAprilTagData(data) {
            const container = document.getElementById('apriltag-data');
            if (!data || data.count === 0) {
                container.innerHTML = '<div class="data-box warning"><div class="data-label">Status</div><div class="data-value">No tags detected</div></div>';
                return;
            }
            
            let html = `
                <div class="data-box">
                    <div class="data-label">Tags Detected</div>
                    <div class="data-value">${data.count}</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Best Tag ID</div>
                    <div class="data-value">${data.bestId}</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Distance</div>
                    <div class="data-value">${data.distance.toFixed(2)}m</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Yaw Angle</div>
                    <div class="data-value">${data.yaw.toFixed(1)}°</div>
                </div>
            `;
            container.innerHTML = html;
        }
        
        function updateObjectData(data) {
            const container = document.getElementById('object-data');
            if (!data || !data.visible) {
                container.innerHTML = '<div class="data-box warning"><div class="data-label">Status</div><div class="data-value">No objects detected</div></div>';
                return;
            }
            
            let html = `
                <div class="data-box">
                    <div class="data-label">Object Visible</div>
                    <div class="data-value">Yes</div>
                </div>`;
                <div class="data-box">
                    <div class="data-label">Yaw</div>
                    <div class="data-value">${data.yaw.toFixed(1)}°</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Pitch</div>
                    <div class="data-value">${data.pitch.toFixed(1)}°</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Area</div>
                    <div class="data-value">${data.area.toFixed(1)}%</div>
                </div>
            `;
            container.innerHTML = html;
        }
        
        function updatePoseData(data) {
            const container = document.getElementById('pose-data');
            if (!data || !data.hasVisionPose) {
                container.innerHTML = '<div class="data-box warning"><div class="data-label">Status</div><div class="data-value">No vision pose available</div></div>';
                return;
            }
            
            let html = `
                <div class="data-box">
                    <div class="data-label">X Position</div>
                    <div class="data-value">${data.x.toFixed(2)}m</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Y Position</div>
                    <div class="data-value">${data.y.toFixed(2)}m</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Rotation</div>
                    <div class="data-value">${data.rotation.toFixed(1)}°</div>
                </div>
                <div class="data-box">
                    <div class="data-label">Timestamp</div>
                    <div class="data-value">${data.timestamp.toFixed(3)}s</div>
                </div>
            `;
            container.innerHTML = html;
        }
        
        // Start auto-update
        updateData();
        updateInterval = setInterval(updateData, 500);
    </script>
</body>
</html>
""";
    }
    
    /**
     * Generate vision data as JSON.
     */
    private String generateVisionJSON() {
        StringBuilder json = new StringBuilder();
        json.append("{");
        
        // Camera data
        json.append("\"cameras\":[");
        json.append("{");
        json.append("\"name\":\"Camera_0\",");
        json.append("\"connected\":").append(SmartDashboard.getBoolean("Vision/Camera_0/Enabled", false)).append(",");
        json.append("\"latency\":").append(SmartDashboard.getNumber("Vision/Camera_0/Latency", 0.0)).append(",");
        json.append("\"fps\":30,");
        json.append("\"hasTargets\":").append(SmartDashboard.getNumber("Vision/AprilTag/Count", 0) > 0).append(",");
        json.append("\"targetCount\":").append((int)SmartDashboard.getNumber("Vision/AprilTag/Count", 0));
        json.append("},");
        json.append("{");
        json.append("\"name\":\"banana_1\",");
        json.append("\"connected\":").append(SmartDashboard.getBoolean("Vision/Camera_1/Enabled", false)).append(",");
        json.append("\"latency\":").append(SmartDashboard.getNumber("Vision/Camera_1/Latency", 0.0)).append(",");
        json.append("\"fps\":30,");
        json.append("\"hasTargets\":").append(SmartDashboard.getBoolean("Vision/Ball/Visible", false)).append(",");
        json.append("\"targetCount\":").append(SmartDashboard.getBoolean("Vision/Ball/Visible", false) ? 1 : 0);
        json.append("}");
        json.append("],");
        
        // AprilTag data
        json.append("\"apriltag\":{");
        json.append("\"count\":").append((int)SmartDashboard.getNumber("Vision/AprilTag/Count", 0)).append(",");
        json.append("\"bestId\":").append((int)SmartDashboard.getNumber("Vision/AprilTag/BestID", -1)).append(",");
        json.append("\"distance\":").append(SmartDashboard.getNumber("Vision/AprilTag/BestDistance", 0.0)).append(",");
        json.append("\"yaw\":").append(SmartDashboard.getNumber("Vision/AprilTag/BestYaw", 0.0));
        json.append("},");
        
        // Object detection data
        json.append("\"object\":{");
        json.append("\"visible\":").append(SmartDashboard.getBoolean("Vision/Ball/Visible", false)).append(",");
        json.append("\"yaw\":").append(SmartDashboard.getNumber("Vision/Ball/Yaw", 0.0)).append(",");
        json.append("\"pitch\":").append(SmartDashboard.getNumber("Vision/Ball/Pitch", 0.0)).append(",");
        json.append("\"area\":").append(SmartDashboard.getNumber("Vision/Ball/Area", 0.0));
        json.append("},");
        
        // Pose estimation data
        json.append("\"pose\":{");
        json.append("\"hasVisionPose\":").append(SmartDashboard.getBoolean("Vision/HasVisionPose", false)).append(",");
        json.append("\"x\":").append(SmartDashboard.getNumber("Vision/EstimatedPose/X", 0.0)).append(",");
        json.append("\"y\":").append(SmartDashboard.getNumber("Vision/EstimatedPose/Y", 0.0)).append(",");
        json.append("\"rotation\":").append(SmartDashboard.getNumber("Vision/EstimatedPose/Rotation", 0.0)).append(",");
        json.append("\"timestamp\":").append(SmartDashboard.getNumber("Vision/PoseTimestamp", 0.0));
        json.append("}");
        
        json.append("}");
        return json.toString();
    }
    
    /**
     * Generate status data as JSON.
     */
    private String generateStatusJSON() {
        return String.format("{\"running\":%b,\"port\":%d}", running, PORT);
    }
}
