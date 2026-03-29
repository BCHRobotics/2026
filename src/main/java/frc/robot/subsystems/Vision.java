/*
 * Vision subsystem for multi-camera AprilTag pose estimation
 * usingPhotonVision, fused with drivetrain odometry
 */

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    // Information about heading and distance to a specific AprilTag
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
    
    // Detailed information about a visible AprilTag
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
    
    // Container class for camera and its associated pose estimator
    private static class CameraModule {
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        public final int index;
        public final String name;
        public PhotonPipelineResult lastResult;
        
        public CameraModule(PhotonCamera camera, PhotonPoseEstimator estimator, int index, String name) {
            this.camera = camera;
            this.poseEstimator = estimator;
            this.index = index;
            this.name = name;
            this.lastResult = null;
        }
    }

    private static class VisionMeasurement {
        public final EstimatedRobotPose estimatedPose;
        public final Matrix<N3, N1> stdDevs;

        public VisionMeasurement(
            EstimatedRobotPose estimatedPose,
            Matrix<N3, N1> stdDevs
        ) {
            this.estimatedPose = estimatedPose;
            this.stdDevs = stdDevs;
        }
    }
    
    // List of active camera modules
    private final List<CameraModule> cameraModules = new ArrayList<>();
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Used to get current odometry pose and update pose estimator with vision measurements
    private final Drivetrain drivetrain;
        
        /**
         * Creates a new Vision subsystem with multi-camera support.
         * Initializes all enabled PhotonVision cameras, loads the 2026 Rebuilt AprilTag
         * field layout, and sets up pose estimators for each camera with their configured transforms.
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
     
                    // Each camera needs its own PhotonPoseEstimator because each camera has a unique physical mounting position and orientation on the robot. 
                    //The pose estimator needs to know exactly where the camera is to correctly convert "what the camera sees" into "where the robot is on the field."
    
                    // Initialize pose estimator with multi-tag strategy for best accuracy
                    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        VisionConstants.kRobotToCams[i]
                    );
                    
                    // Add to active camera list
                    cameraModules.add(new CameraModule(camera, poseEstimator, i, VisionConstants.kCameraNames[i]));
                    
                    System.out.println("Vision: Initialized camera " + i + " (" + VisionConstants.kCameraNames[i] + ")");
                    
                } catch (Exception e) {
                    System.err.println("Vision: Failed to initialize camera " + i + " (" + VisionConstants.kCameraNames[i] + "): " + e.getMessage());
                }
            }
            
            System.out.println("Vision: " + cameraModules.size() + " cameras active");
        }
        
        @Override
        public void periodic() {
        // Process vision measurements from all cameras
        for (CameraModule module : cameraModules) {
            Optional<VisionMeasurement> measurement = getEstimatedGlobalPose(module);
            publishCameraDetectionStatus(module);

            if (measurement.isPresent()) {
                VisionMeasurement accepted = measurement.get();
                Pose2d visionPose = accepted.estimatedPose.estimatedPose.toPose2d();

                drivetrain.addVisionMeasurement(
                    visionPose,
                    accepted.estimatedPose.timestampSeconds,
                    accepted.stdDevs
                );
            }
        }
    }

    private void publishCameraDetectionStatus(CameraModule module) {
        String baseKey = "Vision/" + module.name + "/";

        if (module.lastResult == null) {
            SmartDashboard.putString(baseKey + "DetectionStatus", "NO_DATA");
            SmartDashboard.putNumber(baseKey + "TagCount", 0);
            SmartDashboard.putBoolean(baseKey + "HasTargets", false);
            SmartDashboard.putBoolean(baseKey + "IsMultiTag", false);
            return;
        }

        int tagCount = module.lastResult.getTargets().size();
        String status;
        if (tagCount <= 0) {
            status = "NO_TAGS";
        } else if (tagCount == 1) {
            status = "SINGLE_TAG";
        } else {
            status = "MULTI_TAG";
        }

        SmartDashboard.putString(baseKey + "DetectionStatus", status);
        SmartDashboard.putNumber(baseKey + "TagCount", tagCount);
        SmartDashboard.putBoolean(baseKey + "HasTargets", tagCount > 0);
        SmartDashboard.putBoolean(baseKey + "IsMultiTag", tagCount > 1);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (cameraModules.isEmpty()) {
            return Optional.empty();
        }

        Optional<VisionMeasurement> measurement = getEstimatedGlobalPose(cameraModules.get(0));
        return measurement.map(value -> value.estimatedPose);
    }
    
    /**
     * Gets the latest estimated robot pose from a specific camera.
     * This method retrieves the most recent camera frame, checks for AprilTag targets,
     * and uses the PhotonPoseEstimator to calculate the robot's field position.
     *
    * Filtering:
    * - Rejects estimates with high ambiguity (> threshold)
    * - Requires at least one valid target
    * - Applies a maximum distance cutoff (ignore targets > 3 m)
    * - Validates that pose estimate is reasonable
     * 
     * @param module The camera module to get pose estimate from
     * @return Optional containing the estimated pose if available and valid, empty otherwise
     */
    private Optional<VisionMeasurement> getEstimatedGlobalPose(CameraModule module) {
        // Get latest result from camera (2026 API uses getAllUnreadResults())
        var results = module.camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return Optional.empty();
        }
        
        // Process most recent result
        PhotonPipelineResult result = results.get(results.size() - 1);
        module.lastResult = result;
        
        // Check if we detected any targets
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        
        int tagCount = result.getTargets().size();
        boolean isMultiTag = tagCount > 1;
        double maxAmbiguity = VisionConstants.kMaxAmbiguity;

        // Filter out ambiguous single-tag detections.
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (!isMultiTag && bestTarget.getPoseAmbiguity() > maxAmbiguity) {
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> visionEst = module.poseEstimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = module.poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (visionEst.isEmpty()) {
            return Optional.empty();
        }

        double averageTagDistanceMeters = getAverageTagDistanceMeters(result);
        double maxDistanceMeters = isMultiTag
            ? VisionConstants.kMaxMultiTagDistance
            : VisionConstants.kMaxSingleTagDistance;
        if (averageTagDistanceMeters > maxDistanceMeters) {
            return Optional.empty();
        }

        Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
        Pose2d currentPose = drivetrain.getPose();
        double translationDeltaMeters = estimatedPose.getTranslation().getDistance(currentPose.getTranslation());
        double rotationDeltaDegrees = Math.abs(estimatedPose.getRotation().minus(currentPose.getRotation()).getDegrees());
        double maxTranslationDeltaMeters = isMultiTag
            ? VisionConstants.kMaxMultiTagPoseDeltaMeters
            : VisionConstants.kMaxSingleTagPoseDeltaMeters;
        double maxRotationDeltaDegrees = isMultiTag
            ? VisionConstants.kMaxMultiTagRotationDeltaDegrees
            : VisionConstants.kMaxSingleTagRotationDeltaDegrees;

        if (translationDeltaMeters > maxTranslationDeltaMeters) {
            return Optional.empty();
        }

        if (rotationDeltaDegrees > maxRotationDeltaDegrees) {
            return Optional.empty();
        }

        Matrix<N3, N1> estStdDevs = getEstimationStdDevs(result, averageTagDistanceMeters);

        return Optional.of(new VisionMeasurement(
            visionEst.get(),
            estStdDevs
        ));
    }
    
    /**
     * Computes adaptive vision measurement standard deviations for pose fusion.
     *
     * Adjusts uncertainty based on:
     * - Distance to the detected AprilTag (farther = less reliable)
     * - Number of visible tags (more tags = more reliable)
     *
     * These values are used by the drivetrain pose estimator to weight
     * vision measurements relative to odometry.
     *
     * @param result The camera frame used to generate the measurement
     * @param averageTagDistanceMeters Average camera-to-tag distance in meters
     * @return 3x1 matrix of standard deviations [x, y, theta] in meters/radians
     */
    private Matrix<N3, N1> getEstimationStdDevs(PhotonPipelineResult result, double averageTagDistanceMeters) {
        // Base standard deviations (tuned for your robot)
        double xyStdDev = VisionConstants.kSingleTagStdDevs.get(0, 0);
        double thetaStdDev = VisionConstants.kSingleTagStdDevs.get(2, 0);
        
        // If multiple tags are visible, trust the measurement more
        int numTags = result.getTargets().size();
        if (numTags > 1) {
            xyStdDev = VisionConstants.kMultiTagStdDevs.get(0, 0);
            thetaStdDev = VisionConstants.kMultiTagStdDevs.get(2, 0);
        }

        xyStdDev *= (1 + (averageTagDistanceMeters * averageTagDistanceMeters * VisionConstants.kDistanceWeight));
        thetaStdDev *= (1 + (averageTagDistanceMeters * VisionConstants.kRotationDistanceWeight));
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    private double getAverageTagDistanceMeters(PhotonPipelineResult result) {
        if (!result.hasTargets()) {
            return Double.POSITIVE_INFINITY;
        }

        double totalDistanceMeters = 0.0;
        int measuredTargets = 0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            totalDistanceMeters += cameraToTarget.getTranslation().getNorm();
            measuredTargets++;
        }

        if (measuredTargets == 0) {
            return Double.POSITIVE_INFINITY;
        }

        return totalDistanceMeters / measuredTargets;
    }
    
    /**
     * Gets a specific PhotonCamera instance by index.
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
     * @return List of active camera modules
     */
    public List<CameraModule> getCameraModules() {
        return cameraModules;
    }
    
    /**
     * Gets the number of active cameras.
     * @return Number of enabled and initialized cameras
     */
    public int getActiveCameraCount() {
        return cameraModules.size();
    }
    
    /**
     * Gets the AprilTag field layout.
     * Useful for calculating distances to specific tags or getting tag poses.
     * @return The 2026 Rebuilt AprilTag field layout
     */
    public AprilTagFieldLayout getFieldLayout() {
        return aprilTagFieldLayout;
    }
    
    /**
     * Gets a list of all currently visible AprilTag IDs from all cameras.
     * @return List of visible AprilTag IDs (may contain duplicates if seen by multiple cameras)
     */
    public List<Integer> getVisibleAprilTags() {
        List<Integer> visibleTags = new ArrayList<>();
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.lastResult;
            if (result != null && result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    visibleTags.add(target.getFiducialId());
                }
            }
        }
        
        return visibleTags;
    }
    
    /**
     * Gets detailed information about all currently visible AprilTags.
     * Includes ambiguity values and whether each tag is being used for pose updates.
     * A tag is used for pose updates if its ambiguity is below kMaxAmbiguity threshold.
     * @return List of AprilTagInfo objects with detailed information
     */
    public List<AprilTagInfo> getDetailedAprilTagInfo() {
        List<AprilTagInfo> tagInfoList = new ArrayList<>();
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.lastResult;
            if (result != null && result.hasTargets()) {
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
     * @return Total count of visible targets (may count same tag multiple times if seen by multiple cameras)
     */
    public int getVisibleTagCount() {
        int count = 0;
        
        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.lastResult;
            if (result != null && result.hasTargets()) {
                count += result.getTargets().size();
            }
        }
        
        return count;
    }
    
    /**
     * Gets heading and distance to a specific AprilTag.
     * Calculates the field-relative heading (angle) from the robot's current position
     * to the specified AprilTag, along with the distance.
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
