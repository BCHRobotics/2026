#!/usr/bin/env python3
"""
NetworkTables Vision Monitor
Connects to robot NetworkTables and displays real-time vision, odometry, and camera data.

Usage:
    python nt_vision_monitor.py [--team TEAM_NUMBER] [--ip IP_ADDRESS]

Requirements:
    pip install pynetworktables
"""

import sys
import time
import argparse
import json
from typing import Optional
from http.server import HTTPServer, BaseHTTPRequestHandler
from threading import Thread

try:
    from networktables import NetworkTables
except ImportError:
    print("ERROR: pynetworktables module not found!")
    print("Please install it with: pip install pynetworktables")
    sys.exit(1)



class DiagnosticWebServer(BaseHTTPRequestHandler):
    """HTTP request handler for PhotonVision diagnostics."""
    
    monitor = None  # Will be set to VisionMonitor instance
    
    def log_message(self, format, *args):
        """Suppress request logging."""
        pass
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/' or self.path == '/index.html':
            self.send_html_page()
        elif self.path == '/api/data':
            self.send_json_data()
        elif self.path == '/api/photon':
            self.send_photon_data()
        else:
            self.send_error(404, "Page not found")
    
    def send_html_page(self):
        """Send the main HTML page."""
        html = """
<!DOCTYPE html>
<html>
<head>
    <title>PhotonVision Diagnostics</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #fff;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .subtitle {
            text-align: center;
            margin-bottom: 30px;
            opacity: 0.9;
        }
        .status {
            text-align: center;
            padding: 10px;
            border-radius: 5px;
            margin-bottom: 20px;
            font-weight: bold;
        }
        .connected { background-color: rgba(76, 175, 80, 0.3); }
        .disconnected { background-color: rgba(244, 67, 54, 0.3); }
        .card {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            box-shadow: 0 8px 32px 0 rgba(31, 38, 135, 0.37);
        }
        .card h2 {
            margin-top: 0;
            border-bottom: 2px solid rgba(255,255,255,0.3);
            padding-bottom: 10px;
        }
        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
        }
        .data-item {
            background: rgba(0,0,0,0.2);
            padding: 15px;
            border-radius: 5px;
        }
        .data-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .data-value {
            font-size: 1.5em;
            font-weight: bold;
        }
        .camera-section {
            margin-bottom: 20px;
        }
        .target-item {
            background: rgba(0,0,0,0.3);
            padding: 10px;
            margin: 10px 0;
            border-left: 4px solid #4CAF50;
            border-radius: 5px;
        }
        .no-data {
            text-align: center;
            opacity: 0.6;
            padding: 20px;
        }
        .refresh-info {
            text-align: center;
            opacity: 0.7;
            margin-top: 20px;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>📸 PhotonVision Diagnostics</h1>
        <div class="subtitle">Real-time Camera & Vision Data</div>
        
        <div id="status" class="status disconnected">
            ⏳ Connecting to NetworkTables...
        </div>
        
        <div class="card">
            <h2>Vision Detection</h2>
            <div id="vision-data" class="data-grid">
                <div class="no-data">Loading...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>PhotonVision Cameras</h2>
            <div id="photon-data">
                <div class="no-data">Loading...</div>
            </div>
        </div>
        
        <div class="card">
            <h2>Odometry</h2>
            <div id="odometry-data" class="data-grid">
                <div class="no-data">Loading...</div>
            </div>
        </div>
        
        <div class="refresh-info">
            Auto-refresh every 500ms | Port 8081
        </div>
    </div>
    
    <script>
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    // Update connection status
                    const statusDiv = document.getElementById('status');
                    if (data.connected) {
                        statusDiv.className = 'status connected';
                        statusDiv.innerHTML = '✅ Connected to NetworkTables';
                    } else {
                        statusDiv.className = 'status disconnected';
                        statusDiv.innerHTML = '❌ Disconnected from NetworkTables';
                    }
                    
                    // Update vision data
                    updateVisionData(data.vision);
                    
                    // Update odometry data
                    updateOdometryData(data.odometry);
                })
                .catch(error => {
                    console.error('Error fetching data:', error);
                });
            
            fetch('/api/photon')
                .then(response => response.json())
                .then(data => {
                    updatePhotonData(data);
                })
                .catch(error => {
                    console.error('Error fetching photon data:', error);
                });
        }
        
        function updateVisionData(vision) {
            const visionDiv = document.getElementById('vision-data');
            if (!vision || Object.keys(vision).length === 0) {
                visionDiv.innerHTML = '<div class="no-data">No vision data available</div>';
                return;
            }
            
            let html = '';
            for (const [category, values] of Object.entries(vision)) {
                if (values && Object.keys(values).length > 0) {
                    for (const [key, value] of Object.entries(values)) {
                        html += `
                            <div class="data-item">
                                <div class="data-label">${category} - ${key}</div>
                                <div class="data-value">${value}</div>
                            </div>
                        `;
                    }
                }
            }
            visionDiv.innerHTML = html || '<div class="no-data">No vision detections</div>';
        }
        
        function updatePhotonData(photon) {
            const photonDiv = document.getElementById('photon-data');
            if (!photon || Object.keys(photon).length === 0) {
                photonDiv.innerHTML = '<div class="no-data">No PhotonVision cameras detected</div>';
                return;
            }
            
            let html = '';
            for (const [camera, data] of Object.entries(photon)) {
                html += `<div class="camera-section">`;
                html += `<h3>🎥 ${camera}</h3>`;
                
                if (data && Object.keys(data).length > 0) {
                    html += '<div class="data-grid">';
                    for (const [key, value] of Object.entries(data)) {
                        html += `
                            <div class="data-item">
                                <div class="data-label">${key}</div>
                                <div class="data-value">${value}</div>
                            </div>
                        `;
                    }
                    html += '</div>';
                } else {
                    html += '<div class="no-data">No data from this camera</div>';
                }
                html += '</div>';
            }
            photonDiv.innerHTML = html;
        }
        
        function updateOdometryData(odometry) {
            const odomDiv = document.getElementById('odometry-data');
            if (!odometry || Object.keys(odometry).length === 0) {
                odomDiv.innerHTML = '<div class="no-data">No odometry data available</div>';
                return;
            }
            
            let html = '';
            for (const [key, value] of Object.entries(odometry)) {
                html += `
                    <div class="data-item">
                        <div class="data-label">${key}</div>
                        <div class="data-value">${value}</div>
                    </div>
                `;
            }
            odomDiv.innerHTML = html;
        }
        
        // Update data every 500ms
        setInterval(updateData, 500);
        updateData(); // Initial update
    </script>
</body>
</html>
"""
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        self.wfile.write(html.encode())
    
    def send_json_data(self):
        """Send vision and odometry data as JSON."""
        if not self.monitor:
            self.send_error(500, "Monitor not initialized")
            return
        
        data = {
            'connected': NetworkTables.isConnected(),
            'vision': self.monitor.get_vision_data(),
            'odometry': self.monitor.get_odometry_data()
        }
        
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())
    
    def send_photon_data(self):
        """Send PhotonVision data as JSON."""
        if not self.monitor:
            self.send_error(500, "Monitor not initialized")
            return
        
        data = self.monitor.get_photonvision_data()
        
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())


class VisionMonitor:
    """Monitor and display vision data from NetworkTables."""
    
    def __init__(self, team_number: Optional[int] = None, ip_address: Optional[str] = None):
        """
        Initialize the vision monitor.
        
        Args:
            team_number: FRC team number (e.g., 2026)
            ip_address: Direct IP address of the robot
        """
        # Connect to the robot
        if ip_address:
            print(f"Connecting to robot at {ip_address}...")
            NetworkTables.initialize(server=ip_address)
        elif team_number:
            print(f"Connecting to team {team_number} robot...")
            NetworkTables.initialize(server=f"10.{team_number // 100}.{team_number % 100}.2")
        else:
            print("Connecting to localhost (simulator)...")
            NetworkTables.initialize(server="localhost")
        
        # Get table references
        self.vision_table = NetworkTables.getTable("Vision")
        self.odometry_table = NetworkTables.getTable("Odometry")
        self.photon_table = NetworkTables.getTable("photonvision")
        self.smartdashboard = NetworkTables.getTable("SmartDashboard")
        
        self.running = True
        self.web_server = None
        self.web_server_thread = None
        
    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        """
        Wait for connection to the robot.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if connected, False if timeout
        """
        start_time = time.time()
        while not NetworkTables.isConnected():
            if time.time() - start_time > timeout:
                return False
            time.sleep(0.1)
            print(".", end="", flush=True)
        print("\nConnected!")
        return True
    
    def format_pose(self, table, prefix: str = "") -> str:
        """Format pose data from a NetworkTable."""
        x = table.getNumber(f"{prefix}X", 0.0)
        y = table.getNumber(f"{prefix}Y", 0.0)
        rotation = table.getNumber(f"{prefix}Rotation", 0.0)
        return f"({x:.2f}m, {y:.2f}m, {rotation:.1f}°)"
    
    def get_vision_data(self) -> dict:
        """Retrieve all vision-related data."""
        data = {
            "AprilTag Detection": {},
            "Ball Detection": {},
            "Pose Estimation": {},
            "Camera Status": {}
        }
        
        # AprilTag detection
        april_count = self.vision_table.getNumber("AprilTag/Count", 0)
        data["AprilTag Detection"]["Tags Detected"] = int(april_count)
        
        if april_count > 0:
            data["AprilTag Detection"]["Best Tag ID"] = int(self.vision_table.getNumber("AprilTag/BestID", -1))
            data["AprilTag Detection"]["Best Tag Distance"] = f"{self.vision_table.getNumber('AprilTag/BestDistance', 0.0):.2f}m"
            data["AprilTag Detection"]["Best Tag Yaw"] = f"{self.vision_table.getNumber('AprilTag/BestYaw', 0.0):.1f}°"
        
        # Ball/Object detection
        ball_visible = self.vision_table.getBoolean("Ball/Visible", False)
        data["Ball Detection"]["Ball Visible"] = "Yes" if ball_visible else "No"
        
        if ball_visible:
            data["Ball Detection"]["Yaw"] = f"{self.vision_table.getNumber('Ball/Yaw', 0.0):.1f}°"
            data["Ball Detection"]["Pitch"] = f"{self.vision_table.getNumber('Ball/Pitch', 0.0):.1f}°"
            data["Ball Detection"]["Area"] = f"{self.vision_table.getNumber('Ball/Area', 0.0):.1f}%"
        
        # Pose estimation
        has_vision_pose = self.vision_table.getBoolean("HasVisionPose", False)
        data["Pose Estimation"]["Vision Pose Available"] = "Yes" if has_vision_pose else "No"
        
        if has_vision_pose:
            data["Pose Estimation"]["Estimated Pose"] = self.format_pose(self.vision_table, "EstimatedPose/")
            data["Pose Estimation"]["Timestamp"] = f"{self.vision_table.getNumber('PoseTimestamp', 0.0):.3f}s"
        
        # Camera status
        for i in range(2):  # Check Camera_0 and Camera_1
            cam_name = f"Camera_{i}"
            cam_enabled = self.vision_table.getBoolean(f"{cam_name}/Enabled", False)
            if cam_enabled:
                latency = self.vision_table.getNumber(f"{cam_name}/Latency", 0.0)
                data["Camera Status"][cam_name] = f"Enabled (latency: {latency:.1f}ms)"
        
        return data
    
    def get_odometry_data(self) -> dict:
        """Retrieve odometry data."""
        data = {}
        
        # Robot pose
        data["Robot Pose"] = self.format_pose(self.odometry_table, "Pose/")
        
        # Velocity
        vx = self.odometry_table.getNumber("Velocity/X", 0.0)
        vy = self.odometry_table.getNumber("Velocity/Y", 0.0)
        data["Velocity"] = f"({vx:.2f}m/s, {vy:.2f}m/s)"
        
        # Heading
        heading = self.odometry_table.getNumber("Heading", 0.0)
        data["Heading"] = f"{heading:.1f}°"
        
        return data
    
    def get_photonvision_data(self) -> dict:
        """Retrieve PhotonVision-specific data."""
        data = {}
        
        # Check for camera subtables
        camera_names = ["Camera_0", "banana_1"]  # Based on your config
        
        for cam_name in camera_names:
            cam_table = self.photon_table.getSubTable(cam_name)
            
            # Check if camera has data
            has_targets = cam_table.getBoolean("hasTarget", False)
            if has_targets or cam_table.containsKey("rawBytes"):
                latency = cam_table.getNumber("latencyMillis", 0.0)
                target_count = cam_table.getNumber("targetCount", 0)
                
                data[cam_name] = {
                    "Has Target": "Yes" if has_targets else "No",
                    "Latency": f"{latency:.1f}ms",
                    "Target Count": int(target_count)
                }
                
                if has_targets:
                    yaw = cam_table.getNumber("targetYaw", 0.0)
                    pitch = cam_table.getNumber("targetPitch", 0.0)
                    area = cam_table.getNumber("targetArea", 0.0)
                    
                    data[cam_name]["Target Yaw"] = f"{yaw:.1f}°"
                    data[cam_name]["Target Pitch"] = f"{pitch:.1f}°"
                    data[cam_name]["Target Area"] = f"{area:.1f}%"
        
        return data
    
    def get_smartdashboard_vision_data(self) -> dict:
        """Get vision data that may be on SmartDashboard."""
        data = {}
        
        # Check for common vision keys on SmartDashboard
        keys = self.smartdashboard.getKeys()
        vision_keys = [k for k in keys if any(v in k.lower() for v in ['vision', 'camera', 'april', 'ball', 'target'])]
        
        for key in vision_keys:
            value = self.smartdashboard.getValue(key, None)
            if value is not None:
                if isinstance(value, (int, float)):
                    data[key] = f"{value:.2f}" if isinstance(value, float) else str(value)
                else:
                    data[key] = str(value)
        
        return data
    
    def clear_screen(self):
        """Clear the terminal screen."""
        print("\033[2J\033[H", end="")
    
    def display_data(self):
        """Display all data in a formatted way."""
        self.clear_screen()
        
        print("=" * 80)
        print(" NetworkTables Vision Monitor".center(80))
        print(f" Connected: {'Yes' if NetworkTables.isConnected() else 'No'}".center(80))
        print("=" * 80)
        print()
        
        # Vision Data
        print("📷 VISION DATA")
        print("-" * 80)
        vision_data = self.get_vision_data()
        for category, values in vision_data.items():
            if values:
                print(f"\n  {category}:")
                for key, value in values.items():
                    print(f"    {key:.<30} {value}")
        print()
        
        # Odometry Data
        print("🎯 ODOMETRY DATA")
        print("-" * 80)
        odometry_data = self.get_odometry_data()
        for key, value in odometry_data.items():
            print(f"  {key:.<35} {value}")
        print()
        
        # PhotonVision Data
        print("📸 PHOTONVISION CAMERAS")
        print("-" * 80)
        photon_data = self.get_photonvision_data()
        if photon_data:
            for camera, values in photon_data.items():
                print(f"\n  {camera}:")
                for key, value in values.items():
                    print(f"    {key:.<30} {value}")
        else:
            print("  No PhotonVision data available")
        print()
        
        # SmartDashboard Vision Data
        sd_data = self.get_smartdashboard_vision_data()
        if sd_data:
            print("📊 SMARTDASHBOARD VISION VALUES")
            print("-" * 80)
            for key, value in sd_data.items():
                print(f"  {key:.<35} {value}")
            print()
        
        # Footer
        print("=" * 80)
        print(" Press Ctrl+C to exit".center(80))
        print(" Web interface: http://localhost:8081".center(80))
        print("=" * 80)
    def start_web_server(self, port: int = 8081):
        """
        Start the web server for diagnostics.
        
        Args:
            port: Port to run the web server on (default: 8081)
        """
        try:
            # Set the monitor reference for the handler
            DiagnosticWebServer.monitor = self
            
            self.web_server = HTTPServer(('', port), DiagnosticWebServer)
            self.web_server_thread = Thread(target=self.web_server.serve_forever, daemon=True)
            self.web_server_thread.start()
            
            print(f"🌐 Web server started on http://localhost:{port}")
    def stop(self):
        """Stop the monitor and close connection."""
        self.running = False
        
        # Stop web server
        if self.web_server:
            self.web_server.shutdown()
            print("Web server stopped")
        
        NetworkTables.shutdown()
            print()
    
    def run(self, refresh_rate: float = 0.2, enable_web: bool = True):
    def run(self, refresh_rate: float = 0.2):
        """
        Run the monitor loop.
        
        Args:
            refresh_rate: Time between updates in seconds
        """
        try:
            while self.running:
                self.display_data()
                time.sleep(refresh_rate)
        except KeyboardInterrupt:
            print("\n\nShutting down...")
            self.running = False
    
    def stop(self):
        """Stop the monitor and close connection."""
        self.running = False
        NetworkTables.shutdown()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Monitor vision, odometry, and camera data from robot NetworkTables"
    )
    parser.add_argument(
        "--team",
        type=int,
        help="FRC team number (e.g., 2026)",
        default=None
    )
    parser.add_argument(
        "--ip",
        type=str,
        help="Robot IP address (e.g., 10.20.26.2)",
        default=None
    )
    parser.add_argument(
        "--simulator",
        action="store_true",
        help="Connect to localhost (for robot simulator)"
    )
    parser.add_argument(
        "--no-web",
        action="store_true",
        help="Disable web server interface"
    )
    )
    parser.add_argument(
        "--simulator",
        action="store_true",
        help="Connect to localhost (for robot simulator)"
    )
    
    args = parser.parse_args()
    
    # Determine connection method
    if args.simulator:
        monitor = VisionMonitor()
    elif args.ip:
        monitor = VisionMonitor(ip_address=args.ip)
    elif args.team:
        monitor = VisionMonitor(team_number=args.team)
    else:
        print("No connection method specified. Using localhost (simulator mode).")
        print("Use --team, --ip, or --simulator to specify connection method.")
        print()
        monitor = VisionMonitor()
    
    # Start monitoring
    try:
        monitor.run(refresh_rate=args.rate, enable_web=not args.no_web)
    finally:
        monitor.stop()
    # Start monitoring
    try:
        monitor.run(refresh_rate=args.rate)
    finally:
        monitor.stop()


if __name__ == "__main__":
    main()
