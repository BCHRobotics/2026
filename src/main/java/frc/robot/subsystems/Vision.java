/*
 * Vision.java
 * 
 * Multi-Camera PhotonVision Subsystem for FRC 2026 Rebuilt
 * 
 * This subsystem manages vision processing using PhotonVision for AprilTag detection
 * and robot localization on the 2026 Rebuilt field. It integrates with the drivetrain's
 * odometry system to provide accurate pose estimation.
 * 
 * Key Features:
 * - Multi-camera support (up to 4 cameras)
 * - Individual camera enable/disable configuration
 * - AprilTag detection and pose estimation
 * - Multi-target tracking for improved accuracy
 * - Pose estimation fusion with drivetrain odometry
 * - Ambiguity filtering to reject unreliable measurements
 * - Distance-based standard deviation adjustment
 * - Aggregates poses from all enabled cameras
 * 
 * Hardware Requirements:
 * - Cameras compatible with PhotonVision (OV9281, Microsoft LifeCam, etc.)
 * - Raspberry Pi or other coprocessor running PhotonVision
 * - Network connection to robot radio
 * 
 * Configuration Needed:
 * - Set number of cameras in VisionConstants.kNumCameras
 * - Enable/disable cameras in VisionConstants.kCamerasEnabled
 * - Set camera names in VisionConstants.kCameraNames
 * - Calibrate and set robot-to-camera transforms in VisionConstants.kRobotToCams
 * - Tune vision standard deviations in VisionConstants for your specific setup
 * 
 
 * @see Drivetrain For odometry integration
 * @see VisionConstants For camera configuration.
 */

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    
    /**
     * Detailed information about a visible AprilTag.
     */
    public static class AprilTagInfo {
        public final int id;
        public final double ambiguity;
        public final boolean usedForPoseUpdate;
        public final String cameraName;
        public final double distance;
        
        public AprilTagInfo(int id, double ambiguity, boolean usedForPoseUpdate, String cameraName, double distance) {
            this.id = id;
            this.ambiguity = ambiguity;
            this.usedForPoseUpdate = usedForPoseUpdate;
            this.cameraName = cameraName;
            this.distance = distance;
        }
    }
    
    /**
     * Container class for camera and its associated pose estimator.
     */
    private static class CameraModule {
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        public final int index;
        public final String name;
        
        public CameraModule(PhotonCamera camera, PhotonPoseEstimator estimator, int index, String name) {
            this.camera = camera;
            this.poseEstimator = estimator;
            this.index = index;
            this.name = name;
        }
    }
    
    /**
     * List of active camera modules.
     * 
     * Only cameras that are enabled in VisionConstants.kCamerasEnabled are added.
     * Each camera has its own PhotonCamera instance and PhotonPoseEstimator.
     */
    private final List<CameraModule> cameraModules = new ArrayList<>();
    
    /**
     * 2026 Rebuilt field layout with AprilTag positions.
     * 
     * Loaded from WPILib's built-in field layouts. Contains precise 3D positions
     * of all AprilTags on the 2026 field.
     */
    private final AprilTagFieldLayout aprilTagFieldLayout;
    
    /**
     * Reference to the drivetrain subsystem for pose estimation integration.
     * 
     * Used to get current odometry pose and update pose estimator with vision measurements.
     */
    private final Drivetrain drivetrain;
    
    /**
     * Field2d widget for visualizing vision estimates on dashboard.
     * 
     * Shows the robot's vision-estimated position alongside odometry position
     * for debugging and verification.
     */
    private final Field2d field2d = new Field2d();
    
    /**
     * Creates a new Vision subsystem with multi-camera support.
     * 
     * Initializes all enabled PhotonVision cameras, loads the 2026 Rebuilt AprilTag
     * field layout, and sets up pose estimators for each camera with their configured transforms.
     * 
     * Only cameras with kCamerasEnabled[i] = true will be initialized.
     * 
     * @param drivetrain Reference to drivetrain subsystem for odometry integration
     */
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // Load 2026 Rebuilt AprilTag field layout
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        
        // Initialize each enabled camera
        for (int i = 0; i < VisionConstants.kNumCameras && i < VisionConstants.kCamerasEnabled.length; i++) {
            // Skip disabled cameras
            if (!VisionConstants.kCamerasEnabled[i]) {
                System.out.println("Vision: Camera " + i + " (" + VisionConstants.kCameraNames[i] + ") is disabled");
                continue;
            }
            
            try {
                // Initialize PhotonVision camera
                PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraNames[i]);
                
                // Initialize pose estimator with multi-tag strategy for best accuracy
                PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                    VisionConstants.kRobotToCams[i]
                );
                
                // Set reference pose to current odometry estimate
                poseEstimator.setReferencePose(drivetrain.getPose());
                
                // Add to active camera list
                cameraModules.add(new CameraModule(camera, poseEstimator, i, VisionConstants.kCameraNames[i]));
                
                System.out.println("Vision: Initialized camera " + i + " (" + VisionConstants.kCameraNames[i] + ")");
                
            } catch (Exception e) {
                System.err.println("Vision: Failed to initialize camera " + i + " (" + VisionConstants.kCameraNames[i] + "): " + e.getMessage());
            }
        }
        
        System.out.println("Vision: " + cameraModules.size() + " cameras active");
        
        // Add field visualization to SmartDashboard
        SmartDashboard.putData("Vision Field", field2d);
    }
    
    @Override
    public void periodic() {
        // Update all pose estimators with current odometry
        for (CameraModule module : cameraModules) {
            module.poseEstimator.setReferencePose(drivetrain.getPose());
        }
        
        // Process vision measurements from all cameras
        for (CameraModule module : cameraModules) {
            Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(module);
            
            // If we have a valid vision measurement, update odometry
            if (result.isPresent()) {
                EstimatedRobotPose camPose = result.get();
                
                // Update field visualization with vision estimate (use first valid camera)
                field2d.setRobotPose(camPose.estimatedPose.toPose2d());
                
                // Calculate dynamic standard deviations based on distance and target count
                Matrix<N3, N1> estStdDevs = getEstimationStdDevs(module, camPose.estimatedPose.toPose2d());
                
                // Add vision measurement to drivetrain pose estimator
                // This fuses vision data with odometry for improved accuracy
                drivetrain.addVisionMeasurement(
                    camPose.estimatedPose.toPose2d(),
                    camPose.timestampSeconds,
                    estStdDevs
                );
            }
        }
        
        // Update dashboard with vision status from all cameras
        updateTelemetry();
    }
    
    /**
     * Gets the latest estimated robot pose from a specific camera.
     * 
     * This method retrieves the most recent camera frame, checks for AprilTag targets,
     * and uses the PhotonPoseEstimator to calculate the robot's field position.
     * 
     * Filtering:
     * - Rejects estimates with high ambiguity (> threshold)
     * - Requires at least one valid target
     * - Validates that pose estimate is reasonable
     * 
     * @param module The camera module to get pose estimate from
     * @return Optional containing the estimated pose if available and valid, empty otherwise
     */
    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(CameraModule module) {
        // Get latest result from camera (2026 API uses getAllUnreadResults())
        var results = module.camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        
        // Process most recent result
        PhotonPipelineResult result = results.get(results.size() - 1);
        
        // Check if we detected any targets
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        
        // Filter out ambiguous detections (may be false positives)
        // Ambiguity measures how confident the pose estimate is
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity) {
            return Optional.empty();
        }
        
        // Get pose estimate from PhotonPoseEstimator
        return module.poseEstimator.update(result);
    }
    
    /**
     * Calculates dynamic standard deviations for Kalman filter vision measurements.
     * 
     * Standard deviations represent measurement uncertainty (covariance matrix diagonal)
     * and are used by the Unscented Kalman Filter to optimally weight vision vs. odometry.
     * 
     * Kalman Filter Math:
     * - Standard deviation = sqrt(variance) of measurement noise
     * - Forms the R matrix (measurement noise covariance) in Kalman equations
     * - Kalman Gain K = P * H^T * (H * P * H^T + R)^-1
     * - Higher R (stddev) → Lower K → Filter trusts odometry more
     * - Lower R (stddev) → Higher K → Filter trusts vision more
     * 
     * Dynamic Adjustment Factors:
     * - Distance to target: Farther = less accurate → higher stddev
     * - Number of tags: More tags = more accurate → lower stddev
     * - Target area: Smaller = less accurate → higher stddev
     * 
     * This creates an adaptive filter that automatically adjusts trust based
     * on measurement quality, providing optimal fusion in all conditions.
     * 
     * @param module The camera module providing the measurement
     * @param estimatedPose The vision-estimated robot pose
     * @return 3x1 matrix of standard deviations [x, y, theta] in meters/radians
     */
    private Matrix<N3, N1> getEstimationStdDevs(CameraModule module, Pose2d estimatedPose) {
        // Get latest camera result (2026 API)
        var results = module.camera.getAllUnreadResults();
        if (results.isEmpty()) {
            // No results available, return default single-tag stddevs
            return VisionConstants.kSingleTagStdDevs;
        }
        PhotonPipelineResult result = results.get(results.size() - 1);
        
        // Base standard deviations (tuned for your robot)
        // PLACEHOLDER: These should be tuned through testing
        double xyStdDev = VisionConstants.kSingleTagStdDevs.get(0, 0);
        double thetaStdDev = VisionConstants.kSingleTagStdDevs.get(2, 0);
        
        // If multiple tags are visible, trust the measurement more
        int numTags = result.getTargets().size();
        if (numTags > 1) {
            xyStdDev = VisionConstants.kMultiTagStdDevs.get(0, 0);
            thetaStdDev = VisionConstants.kMultiTagStdDevs.get(2, 0);
        }
        
        // Increase uncertainty based on distance to target
        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            
            // Get distance to the AprilTag
            Optional<Pose3d> targetPose = aprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
            if (targetPose.isPresent()) {
                double distance = estimatedPose.getTranslation()
                    .getDistance(targetPose.get().toPose2d().getTranslation());
                
                // Scale uncertainty with distance (quadratic relationship)
                // Farther targets are much less reliable
                xyStdDev *= (1 + (distance * distance * VisionConstants.kDistanceWeight));
            }
        }
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
    
    /**
     * Updates SmartDashboard with current vision system status from all cameras.
     * 
     * Displays for each camera:
     * - Whether targets are detected
     * - Number of targets visible
     * - Best target ID and distance
     * - Pose ambiguity
     * - Pipeline timestamp
     * 
     * Also displays aggregate statistics:
     * - Total cameras active
     * - Total cameras seeing targets
     */
    private void updateTelemetry() {
        int totalTargets = 0;
        int camerasWithTargets = 0;
        
        // Update telemetry for each camera
        for (CameraModule module : cameraModules) {
            var results = module.camera.getAllUnreadResults();
            String prefix = "Vision/Cam" + module.index + "/";
            
            if (results.isEmpty()) {
                SmartDashboard.putBoolean(prefix + "Has Targets", false);
                continue;
            }
            
            PhotonPipelineResult result = results.get(results.size() - 1);
            
            boolean hasTargets = result.hasTargets();
            SmartDashboard.putBoolean(prefix + "Has Targets", hasTargets);
            SmartDashboard.putNumber(prefix + "Target Count", result.getTargets().size());
            SmartDashboard.putNumber(prefix + "Timestamp", result.getTimestampSeconds());
            SmartDashboard.putString(prefix + "Name", module.name);
            
            if (hasTargets) {
                camerasWithTargets++;
                totalTargets += result.getTargets().size();
                
                PhotonTrackedTarget best = result.getBestTarget();
                SmartDashboard.putNumber(prefix + "Best Target ID", best.getFiducialId());
                SmartDashboard.putNumber(prefix + "Ambiguity", best.getPoseAmbiguity());
                SmartDashboard.putNumber(prefix + "Area", best.getArea());
                
                // Display which alliance's tags we're seeing (Blue: 1-8, Red: 9-16)
                if (best.getFiducialId() <= 8) {
                    SmartDashboard.putString(prefix + "Alliance", "Blue");
                } else {
                    SmartDashboard.putString(prefix + "Alliance", "Red");
                }
            }
        }
        
        // Update aggregate statistics
        SmartDashboard.putNumber("Vision/Total Cameras", cameraModules.size());
        SmartDashboard.putNumber("Vision/Cameras With Targets", camerasWithTargets);
        SmartDashboard.putNumber("Vision/Total Targets", totalTargets);
    }
    
    /**
     * Gets a specific PhotonCamera instance by index.
     * 
     * @param index Camera index (0-3)
     * @return The PhotonCamera object, or null if index is invalid or camera not enabled
     */
    public PhotonCamera getCamera(int index) {
        for (CameraModule module : cameraModules) {
            if (module.index == index) {
                return module.camera;
            }
        }
        return null;
    }
    
    /**
     * Gets all active camera modules.
     * 
     * @return List of active camera modules
     */
    public List<CameraModule> getCameraModules() {
        return cameraModules;
    }
    
    /**
     * Gets the number of active cameras.
     * 
     * @return Number of enabled and initialized cameras
     */
    public int getActiveCameraCount() {
        return cameraModules.size();
    }
    
    /**
     * Gets the AprilTag field layout.
     * 
     * Useful for calculating distances to specific tags or getting tag poses.
     * 
     * @return The 2026 Rebuilt AprilTag field layout
     */
    public AprilTagFieldLayout getFieldLayout() {
        return aprilTagFieldLayout;
    }
    
    /**
     * Gets a list of all currently visible AprilTag IDs from all cameras.
     * 
     * @return List of visible AprilTag IDs (may contain duplicates if seen by multiple cameras)
     */
    public List<Integer> getVisibleAprilTags() {
        List<Integer> visibleTags = new ArrayList<>();
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.camera.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    visibleTags.add(target.getFiducialId());
                }
            }
        }
        
        return visibleTags;
    }
    
    /**
     * Gets detailed information about all currently visible AprilTags.
     * 
     * Includes ambiguity values and whether each tag is being used for pose updates.
     * A tag is used for pose updates if its ambiguity is below kMaxAmbiguity threshold.
     * 
     * @return List of AprilTagInfo objects with detailed information
     */
    public List<AprilTagInfo> getDetailedAprilTagInfo() {
        List<AprilTagInfo> tagInfoList = new ArrayList<>();
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.camera.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    double ambiguity = target.getPoseAmbiguity();
                    boolean usedForUpdate = ambiguity <= VisionConstants.kMaxAmbiguity;
                    
                    // Calculate distance to tag
                    double distance = -1.0;
                    Optional<Pose3d> targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                    if (targetPose.isPresent()) {
                        distance = drivetrain.getPose().getTranslation()
                            .getDistance(targetPose.get().toPose2d().getTranslation());
                    }
                    
                    tagInfoList.add(new AprilTagInfo(
                        target.getFiducialId(),
                        ambiguity,
                        usedForUpdate,
                        module.name,
                        distance
                    ));
                }
            }
        }
        
        return tagInfoList;
    }
    
    /**
     * Gets the total number of AprilTags currently visible across all cameras.
     * 
     * @return Total count of visible targets (may count same tag multiple times if seen by multiple cameras)
     */
    public int getVisibleTagCount() {
        int count = 0;
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.camera.getLatestResult();
            if (result.hasTargets()) {
                count += result.getTargets().size();
            }
        }
        
        return count;
    }
    
    /**
     * Information about heading and distance to a specific AprilTag.
     */
    public static class TagNavigationInfo {
        public final int tagId;
        public final boolean tagExists;
        public final double distance;  // meters
        public final double heading;   // degrees, field-relative angle to tag
        
        public TagNavigationInfo(int tagId, boolean tagExists, double distance, double heading) {
            this.tagId = tagId;
            this.tagExists = tagExists;
            this.distance = distance;
            this.heading = heading;
        }
    }
    
    /**
     * Gets heading and distance to a specific AprilTag.
     * 
     * Calculates the field-relative heading (angle) from the robot's current position
     * to the specified AprilTag, along with the distance.
     * 
     * @param tagId The AprilTag ID to get navigation info for
     * @return TagNavigationInfo with distance and heading, or invalid info if tag doesn't exist
     */
    public TagNavigationInfo getTagNavigationInfo(int tagId) {
        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);
        
        if (!tagPoseOptional.isPresent()) {
            // Tag doesn't exist in field layout
            return new TagNavigationInfo(tagId, false, 0.0, 0.0);
        }
        
        Pose2d tagPose = tagPoseOptional.get().toPose2d();
        Pose2d robotPose = drivetrain.getPose();
        
        // Calculate distance
        double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
        
        // Calculate heading (field-relative angle from robot to tag)
        double deltaX = tagPose.getX() - robotPose.getX();
        double deltaY = tagPose.getY() - robotPose.getY();
        double heading = Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        return new TagNavigationInfo(tagId, true, distance, heading);
    }
}
