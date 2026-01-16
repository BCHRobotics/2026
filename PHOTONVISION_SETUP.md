# PhotonVision Setup Guide - 2026 Rebuilt

This guide walks you through setting up PhotonVision for AprilTag detection and pose estimation on your FRC robot for the 2026 Rebuilt game.

## Table of Contents
1. [Hardware Requirements](#hardware-requirements)
2. [PhotonVision Installation](#photonvision-installation)
3. [Camera Configuration](#camera-configuration)
4. [Robot Code Configuration](#robot-code-configuration)
5. [Calibration and Tuning](#calibration-and-tuning)
6. [Testing and Verification](#testing-and-verification)
7. [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Required Components
- **Camera**: OV9281, Microsoft LifeCam HD-3000, or other PhotonVision-compatible camera
  - Recommended: OV9281 (global shutter, good low-light performance)
  - Alternative: Arducam OV9281 USB Camera Module
  
- **Coprocessor**: Raspberry Pi 4/5, Orange Pi, or similar
  - Minimum: Raspberry Pi 4 (4GB RAM recommended)
  - Storage: 32GB microSD card (Class 10 or better)
  
- **Power**: Reliable 5V power supply for coprocessor
  - Use VRM or similar regulated power source
  - Do NOT power from USB ports on roboRIO
  
- **Networking**: Ethernet cable or reliable WiFi connection to robot radio

### Mounting Considerations
- Mount camera with clear view of field
- Typical positions: front-center of robot, elevated for better tag visibility
- Minimize vibration (use soft mounts or foam)
- Ensure camera stays clean during matches (consider protective cover)

---

## PhotonVision Installation

### 1. Download PhotonVision
1. Go to: https://photonvision.org/
2. Download the latest stable release for your coprocessor
3. Choose the appropriate image:
   - Raspberry Pi: `.xz` image file
   - Other: Follow platform-specific instructions

### 2. Flash to SD Card
1. Download Balena Etcher: https://www.balena.io/etcher/
2. Insert microSD card into your computer
3. Open Balena Etcher
4. Select the PhotonVision image
5. Select your SD card
6. Flash (takes 5-10 minutes)

### 3. First Boot
1. Insert SD card into Raspberry Pi
2. Connect camera via USB
3. Power on the Pi (via VRM or 5V power supply)
4. Wait 1-2 minutes for boot
5. Connect computer to robot network

### 4. Access PhotonVision UI
1. Open web browser
2. Navigate to: `http://photonvision.local:5800`
   - If that doesn't work, try: `http://10.TE.AM.11:5800` (replace TE.AM with your team number)
   - Example for team 1678: `http://10.16.78.11:5800`

---

## Camera Configuration

### 1. Set Camera Name
1. In PhotonVision UI, go to **Settings** tab
2. Select your camera from the dropdown
3. Set a descriptive name (e.g., "OV9281", "Front_Camera")
4. **IMPORTANT**: Update this name in `Constants.java`:
   ```java
   public static final String kCameraName = "YOUR_CAMERA_NAME_HERE";
   ```

### 2. Configure Pipeline for AprilTags
1. Create new pipeline: Click **"+"** button
2. Set pipeline type: **AprilTag**
3. Configure settings:
   - **Resolution**: 1280x720 (good balance) or 640x480 (faster)
   - **FPS**: 30-60 (higher is better, but depends on camera)
   - **Exposure**: Auto or manual (tune for your lighting)
   - **Tag Family**: 36h11 (standard for FRC 2026)

### 3. Camera Calibration
**Critical for accurate pose estimation!**

1. Download calibration board:
   - PhotonVision provides printable calibration patterns
   - Print on flat surface (foam board recommended)
   
2. Run calibration in PhotonVision UI:
   - Go to **Cameras** tab
   - Click **Calibrate**
   - Follow on-screen instructions
   - Take 12-25 images at various angles and distances
   - Process calibration (takes 1-2 minutes)
   
3. **Verify calibration**:
   - View a flat surface (like a wall)
   - Lines should appear straight, not curved
   - If distorted, re-calibrate

---

## Robot Code Configuration

### 1. Measure Camera Transform
**Most important step for accurate pose estimation!**

Measure from robot center to camera:

```
Translation (meters):
  X: Forward/backward (+ = forward from robot center)
  Y: Left/right (+ = left from robot center)  
  Z: Up/down (+ = up from ground)

Rotation (radians):
  Roll: Rotation around X axis (usually 0)
  Pitch: Tilt up/down (- = tilted up, typical for cameras)
  Yaw: Rotation around Z axis (usually 0 if camera faces forward)
```

**Example**: Camera mounted 12 inches forward, 6 inches up, tilted 15° up:
```java
public static final Transform3d kRobotToCam = new Transform3d(
    new Translation3d(
        Units.inchesToMeters(12),  // 12 inches forward
        Units.inchesToMeters(0),   // 0 inches left/right (centered)
        Units.inchesToMeters(6)    // 6 inches up
    ),
    new Rotation3d(
        0,                          // No roll
        -Math.toRadians(15),        // 15 degrees tilt up (negative!)
        0                           // No yaw
    )
);
```

**Update in `Constants.java`** → `VisionConstants.kRobotToCam`

### 2. Configure Network Settings
Ensure robot can reach coprocessor:

- **mDNS**: `photonvision.local` (should work automatically)
- **Static IP**: `10.TE.AM.11` (if mDNS fails)
  - Configure in PhotonVision Settings → Networking

---

## Calibration and Tuning

### 1. Test Basic Detection
1. Deploy code to robot
2. Position robot 6-10 feet from an AprilTag
3. Check SmartDashboard:
   - `Vision/Has Targets` should be `true`
   - `Vision/Target Count` should be `> 0`
   - `Vision/Best Target ID` should match the tag you're viewing

### 2. Verify Pose Estimation
1. Drive robot to **known position** on field
2. Reset odometry to that position
3. Observe `Vision Field` widget on SmartDashboard
4. Vision estimate should be close to odometry (within 0.5m initially)

### 3. Tune Standard Deviations
**Goal**: Balance between trusting vision vs. odometry

1. **Single Tag Testing**:
   - View one AprilTag from 3-6 feet
   - Observe position error
   - Adjust `kSingleTagStdDevs` in `Constants.java`:
     - If vision is accurate: **decrease** values (trust more)
     - If vision is jittery: **increase** values (trust less)
   
2. **Multi Tag Testing**:
   - Position robot to see 2+ tags
   - Should be more stable than single tag
   - Tune `kMultiTagStdDevs` (typically 1/3 to 1/5 of single-tag)

3. **Distance Weighting**:
   - Test from various distances (3-20 feet)
   - Adjust `kDistanceWeight`:
     - If far targets are unreliable: **increase** value
     - If rejecting too many measurements: **decrease** value

### 4. Tune Ambiguity Threshold
- Watch `Vision/Best Target Ambiguity` on dashboard
- Typical good values: 0.05 - 0.15
- Adjust `kMaxAmbiguity`:
  - If false positives occur: **decrease** (more strict)
  - If rejecting good detections: **increase** (less strict)

### 5. Tune Vision Alignment PIDs
For autonomous alignment to AprilTags:

1. Test approach to tag
2. Tune `kAlignP` (X-axis movement):
   - Start low (0.3)
   - Increase until responsive but not oscillating
   
3. Tune `kRotP` (rotation):
   - Start low (0.01)
   - Increase until quick response without overshoot

---

## Testing and Verification

### Pre-Match Checklist
- [ ] PhotonVision boots and connects to robot
- [ ] Camera name matches code
- [ ] Camera transform is accurate
- [ ] Detects tags from 3-20 feet
- [ ] Pose estimates are reasonable (< 0.5m error)
- [ ] Multiple tags improve accuracy
- [ ] No excessive jitter in estimates
- [ ] Dashboard shows vision data

### Field Testing Procedure
1. **Position Test**:
   - Place robot at known field position
   - Reset odometry
   - Check vision estimate matches (within tuned tolerances)

2. **Movement Test**:
   - Drive around field
   - Watch pose estimate on dashboard
   - Should not "jump" or reset unexpectedly

3. **Tag Coverage Test**:
   - Drive to various field positions
   - Verify at least 1 tag visible from most locations
   - Identify "blind spots" where no tags are visible

### 2026 Rebuilt Tag Locations
- **Blue Alliance**: Tags 1-8
- **Red Alliance**: Tags 9-16

Common tag IDs:
- **Source/Loading Zones**: Specific tags vary, check game manual
- **Scoring Positions**: Reef, processor, etc.
- **Field Perimeter**: Corner and wall tags for localization

---

## Troubleshooting

### Camera Not Detected
- Check USB connection
- Try different USB port on coprocessor
- Verify camera is supported: https://docs.photonvision.org/en/latest/docs/hardware/index.html
- Check power supply to coprocessor

### Can't Access PhotonVision UI
- Verify computer is on robot network
- Try IP address instead of mDNS: `http://10.TE.AM.11:5800`
- Check robot radio is powered and working
- Verify coprocessor has power (look for LED activity)

### No Targets Detected
- Verify pipeline is set to AprilTag mode
- Check lighting conditions (too bright or too dark)
- Adjust camera exposure
- Ensure AprilTags are properly printed and mounted
- Check if camera lens is clean
- Verify correct tag family (36h11 for FRC 2026)

### Pose Estimates are Inaccurate
- **Recalibrate camera** (most common issue)
- Verify `kRobotToCam` transform is correct
- Check for camera vibration or movement
- Tune standard deviations
- Increase `kMaxAmbiguity` threshold (may be rejecting good data)

### Vision Estimates "Jump" or Reset
- Increase standard deviations (trust vision less)
- Check `kMaxAmbiguity` (may be accepting bad data)
- Verify tags are properly mounted (not moving)
- Check for multi-path reflections (shiny surfaces)

### Poor Performance at Distance
- Increase `kDistanceWeight` to trust far measurements less
- Improve lighting (add LED ring light around camera)
- Use higher resolution camera or setting
- Consider multiple cameras for better coverage

### Code Doesn't Compile
- Ensure PhotonLib is installed:
  - Open WPILib VS Code
  - Press Ctrl+Shift+P
  - Type "Manage Vendor Libraries"
  - Select "Install new libraries (online)"
  - Check "PhotonLib"
- Verify all imports are correct
- Check vendordeps folder contains `photonlib.json`

---

## Additional Resources

### Official Documentation
- PhotonVision Docs: https://docs.photonvision.org/
- WPILib Pose Estimation: https://docs.wpilib.org/en/stable/docs/software/vision/pose-estimation.html
- 2026 Game Manual: https://www.firstinspires.org/resource-library/frc/competition-manual-qa-system

### Community Resources
- Chief Delphi: https://www.chiefdelphi.com/
- PhotonVision Discord: (link in PhotonVision docs)
- FRC Discord: https://discord.gg/frc

### Debugging Tools
- PhotonVision UI: Real-time camera view and pipeline tuning
- SmartDashboard/Shuffleboard: Monitor vision data during matches
- AdvantageScope: Replay and analyze logs with vision data

---

## Quick Reference - Important Placeholders to Configure

| Constant | Location | What to Set | Example |
|----------|----------|-------------|---------|
| `kCameraName` | Constants.java | PhotonVision camera name | `"OV9281"` |
| `kRobotToCam` | Constants.java | Robot to camera transform | See measurement guide |
| `kMaxAmbiguity` | Constants.java | Ambiguity threshold | `0.2` (start) |
| `kSingleTagStdDevs` | Constants.java | Single tag trust values | `[0.7, 0.7, 0.9]` |
| `kMultiTagStdDevs` | Constants.java | Multi tag trust values | `[0.2, 0.2, 0.3]` |
| `kDistanceWeight` | Constants.java | Distance scaling factor | `0.01` |

---

**Good luck with your 2026 season! 🤖**
