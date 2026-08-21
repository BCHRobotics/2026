/*
 * Vision subsystem for multi-camera AprilTag pose estimation using PhotonVision,
 * fused with the drivetrain's pose estimator.
 *
 * ============================================================================
 * v2 ACCURACY UPDATE — WHAT CHANGED AND WHY (read me, future students!)
 * ============================================================================
 * Last season the robot's vision pose was "jumpy": most of the time it was
 * great, but sometimes it drifted a couple of inches for no obvious reason.
 * We traced the jumpiness to three habits in the old code. Each fix below is
 * tagged "CHANGE #n" so you can find it with Ctrl+F.
 *
 * CHANGE #1 — We now process EVERY frame from each camera, not just the newest.
 *   The cameras shoot ~90 pictures per second but the robot loop runs 50 times
 *   per second. The old code grabbed only the newest picture and threw the rest
 *   away — literally deleting half our measurements. More measurements = smoother,
 *   more repeatable pose.
 *
 * CHANGE #2 — Soft limits instead of hard walls.
 *   The old code threw away a whole measurement the instant it crossed a limit
 *   (too ambiguous, too far). That caused "vision starvation": near the cage the
 *   tags are partly blocked, measurements hovered right at the limits, and got
 *   rejected over and over — leaving us on dead-reckoned odometry exactly when
 *   we needed precision for the climb. Now, instead of a wall there is a
 *   "trust ramp": as a measurement gets worse we believe it LESS (we inflate its
 *   standard deviation — see CHANGE #3) but we still use it. Only truly bad
 *   data (way past the limits, or a "teleport" that disagrees with where we know
 *   we are) gets dropped completely.
 *
 * CHANGE #3 — Vision no longer steers our HEADING (rotation).
 *   A single AprilTag seen at a distance can look like the robot is facing two
 *   different directions (that's what "ambiguity" measures). The old code fed
 *   that shaky heading number into the pose estimator, so a flickering tag could
 *   wobble the whole pose. Now single-tag measurements only correct X/Y
 *   position; we give them an INFINITE heading uncertainty so the estimator
 *   ignores their rotation completely. Heading is the gyro's job — the NavX is
 *   excellent at rotation and never blinks. (Multi-tag shots see several tags
 *   at once, which locks down rotation reliably, so those still contribute.)
 *
 * CHANGE #4 — Startup logging of the field layout and library versions.
 *   There are TWO official 2026 field layouts (welded vs. AndyMark). If the
 *   RoboRIO code and the PhotonVision coprocessors disagree about tag positions,
 *   every pose comes out slightly wrong in a way that looks random. We now log
 *   exactly which layout this code expects so it can be checked against the
 *   coprocessor settings at the pit station. (Checking the coprocessors
 *   themselves is a driver-station task — see REVIEW.md section A1.)
 *
 * Think of the pose estimator like a jury: odometry testifies every cycle,
 * each vision frame is a witness. The old code silenced witnesses and then
 * convicted others outright. Now every witness testifies, and sketchy ones
 * are simply believed less.
 * ============================================================================
 */

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private static final String TUNING_KEY_PREFIX = "Vision/Tuning/";

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

    // Container class for a camera and its associated pose estimator
    private static class CameraModule {
        public final PhotonCamera camera;
        public final PhotonPoseEstimator poseEstimator;
        public final int index;
        public final String name;
        // Telemetry: the newest raw frame (for dashboards) and running stats.
        public PhotonPipelineResult lastResult;
        public String lastRejectReason;
        public int rejectedFrameCount = 0;
        // CHANGE #1: counters now track ALL frames, not just the surviving one.
        public int processedFrameCount = 0;
        public int acceptedFrameCount = 0;

        public CameraModule(PhotonCamera camera, PhotonPoseEstimator estimator, int index, String name) {
            this.camera = camera;
            this.poseEstimator = estimator;
            this.index = index;
            this.name = name;
            this.lastResult = null;
            this.lastRejectReason = "No frame processed yet";
        }
    }

    // One accepted pose measurement, ready to hand to the drivetrain's estimator.
    private static class VisionMeasurement {
        public final EstimatedRobotPose estimatedPose;
        public final Matrix<N3, N1> stdDevs;
        public final int tagCount;
        public final double averageTagDistanceMeters;

        public VisionMeasurement(
            EstimatedRobotPose estimatedPose,
            Matrix<N3, N1> stdDevs,
            int tagCount,
            double averageTagDistanceMeters
        ) {
            this.estimatedPose = estimatedPose;
            this.stdDevs = stdDevs;
            this.tagCount = tagCount;
            this.averageTagDistanceMeters = averageTagDistanceMeters;
        }
    }

    // How long we consider the vision pose "fresh" for dashboard purposes.
    private static final double VISION_TIMEOUT_SECONDS = 0.2;
    private double lastVisionMeasurementTimestamp = Double.NEGATIVE_INFINITY;

    // List of active camera modules
    private final List<CameraModule> cameraModules = new ArrayList<>();
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Used to get current odometry pose and update pose estimator with vision measurements
    private final Drivetrain drivetrain;

    // Field2d widget for visualizing vision estimates on the dashboard.
    private final Field2d field2d = new Field2d();

    /**
     * Creates a new Vision subsystem with multi-camera support.
     */
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // CHANGE #4: say OUT LOUD (well, in the logs) which field layout we expect.
        // If the PhotonVision coprocessors were left on the default "welded" layout
        // while this code loads the AndyMark layout, every pose is biased and the
        // error looks random. Compare this log line against the layout shown in each
        // coprocessor's Settings tab before blaming the math.
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        System.out.println("Vision: [layout-check] RIO code expects AprilTag layout '2026-rebuilt-andymark'. "
            + "Verify each PhotonVision coprocessor is set to the SAME layout!");
        Logger.recordOutput("Vision/ExpectedFieldLayout", "2026-rebuilt-andymark");
        Logger.recordOutput("Vision/PhotonLibVersion", org.photonvision.PhotonVersion.versionString);

        // Initialize each enabled camera
        for (int i = 0; i < VisionConstants.kNumCameras && i < VisionConstants.kCamerasEnabled.length; i++) {
            if (!VisionConstants.kCamerasEnabled[i]) {
                System.out.println("Vision: Camera " + i + " (" + VisionConstants.kCameraNames[i] + ") is disabled");
                continue;
            }

            try {
                PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraNames[i]);

                // Each camera needs its own PhotonPoseEstimator because each camera
                // sits in a different spot on the robot. The estimator turns "what
                // the camera sees" into "where the robot is on the field".
                PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    VisionConstants.kRobotToCams[i]
                );

                cameraModules.add(new CameraModule(camera, poseEstimator, i, VisionConstants.kCameraNames[i]));
                System.out.println("Vision: Initialized camera " + i + " (" + VisionConstants.kCameraNames[i] + ")");

            } catch (Exception e) {
                System.err.println("Vision: Failed to initialize camera " + i + " ("
                    + VisionConstants.kCameraNames[i] + "): " + e.getMessage());
            }
        }

        System.out.println("Vision: " + cameraModules.size() + " cameras active");
        SmartDashboard.putData("Vision/Field", field2d);
        publishTuningDefaults();
    }

    @Override
    public void periodic() {
        boolean acceptedMeasurement = false;
        List<Pose3d> acceptedRobotPoses = new ArrayList<>();
        int lastAcceptedTagCount = 0;
        double lastAcceptedAverageTagDistanceMeters = 0.0;

        // =====================================================================
        // CHANGE #1: process EVERY unread frame from every camera.
        //
        // getAllUnreadResults() hands back all frames that arrived since the last
        // time we asked (usually 1-2 per loop, sometimes more). The old code kept
        // only the last element of that list. Frames arrive in time order, so we
        // walk the list from oldest to newest and give EACH accepted frame to the
        // pose estimator with its own timestamp. The estimator knows how to
        // rewind time and apply delayed corrections correctly — that's its job.
        // =====================================================================
        for (CameraModule module : cameraModules) {
            List<PhotonPipelineResult> results = module.camera.getAllUnreadResults();

            for (PhotonPipelineResult result : results) {
                module.processedFrameCount++;
                module.lastResult = result; // dashboards show the newest frame

                Optional<VisionMeasurement> measurement = evaluateFrame(module, result);

                if (measurement.isPresent()) {
                    VisionMeasurement accepted = measurement.get();
                    Pose2d visionPose = accepted.estimatedPose.estimatedPose.toPose2d();

                    field2d.setRobotPose(visionPose);
                    drivetrain.addVisionMeasurement(
                        visionPose,
                        accepted.estimatedPose.timestampSeconds,
                        accepted.stdDevs
                    );

                    acceptedRobotPoses.add(accepted.estimatedPose.estimatedPose);
                    lastAcceptedTagCount = accepted.tagCount;
                    lastAcceptedAverageTagDistanceMeters = accepted.averageTagDistanceMeters;
                    module.acceptedFrameCount++;
                    module.lastRejectReason = "Accepted";
                    acceptedMeasurement = true;
                    lastVisionMeasurementTimestamp = Timer.getFPGATimestamp();
                }

                Logger.recordOutput("Vision/" + module.name + "/AppliedXYStdDev",
                    measurement.isPresent() ? measurement.get().stdDevs.get(0, 0) : -1.0);
                Logger.recordOutput("Vision/" + module.name + "/AppliedThetaStdDev",
                    measurement.isPresent() ? measurement.get().stdDevs.get(2, 0) : -1.0);
            }

            SmartDashboard.putString("Vision/" + module.name + "/LastRejectReason", module.lastRejectReason);
        }

        // ---- Telemetry (kept from v1 so existing dashboards keep working) ----
        List<Pose3d> detectedTagFieldPoses = getDetectedTagFieldPoses();
        List<Pose3d> cameraFieldPoses = getCameraFieldPoses();
        Logger.recordOutput("Vision/RobotPoses", acceptedRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/CameraPoses", cameraFieldPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/TagPoses", detectedTagFieldPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/VisibleTagCount", getVisibleTagCount());
        Logger.recordOutput("Vision/VisibleTagIds", getVisibleAprilTags().stream().mapToInt(Integer::intValue).toArray());
        boolean hasVisionPose = (Timer.getFPGATimestamp() - lastVisionMeasurementTimestamp) < VISION_TIMEOUT_SECONDS;
        Logger.recordOutput("Vision/HasVisionPose", hasVisionPose);
        Logger.recordOutput("Vision/LastAcceptedTagCount", lastAcceptedTagCount);
        Logger.recordOutput("Vision/LastAcceptedAvgTagDistanceMeters", lastAcceptedAverageTagDistanceMeters);

        for (CameraModule module : cameraModules) {
            Logger.recordOutput("Vision/" + module.name + "/HasTargets",
                module.lastResult != null && module.lastResult.hasTargets());
            Logger.recordOutput("Vision/" + module.name + "/LastRejectReason", module.lastRejectReason);
            Logger.recordOutput("Vision/" + module.name + "/TotalRejectedFrames", module.rejectedFrameCount);
            // CHANGE #1 telemetry: processed vs accepted tells you the trust ramp's health.
            // If "processed" climbs but "accepted" stalls, vision is starving — check lighting/exposure.
            Logger.recordOutput("Vision/" + module.name + "/FramesProcessed", module.processedFrameCount);
            Logger.recordOutput("Vision/" + module.name + "/FramesAccepted", module.acceptedFrameCount);
            if (module.lastResult != null) {
                Logger.recordOutput("Vision/" + module.name + "/LatencyMillis",
                    (Timer.getFPGATimestamp() - module.lastResult.getTimestampSeconds()) * 1000.0);
                Logger.recordOutput("Vision/" + module.name + "/FrameTimestamp",
                    module.lastResult.getTimestampSeconds());
            }
        }

        SmartDashboard.putBoolean("Vision/HasVisionPose", acceptedMeasurement);

        Logger.recordOutput("Vision/Tuning/MaxAmbiguity", getMaxAmbiguity());
        Logger.recordOutput("Vision/Tuning/AmbiguityRejectLimit", VisionConstants.kAmbiguityRejectLimit);
        // CHANGE #5: make the heading-authority choice visible in every log.
        Logger.recordOutput("Vision/GyroIsHeadingAuthority", !VisionConstants.kMultiTagHeadingEnabled);
        Logger.recordOutput("Vision/Tuning/SingleTagXYStdDev", getSingleTagXYStdDev());
        Logger.recordOutput("Vision/Tuning/MultiTagXYStdDev", getMultiTagXYStdDev());
        Logger.recordOutput("Vision/Tuning/MultiTagThetaStdDev", getMultiTagThetaStdDev());
        Logger.recordOutput("Vision/Tuning/DistanceWeight", getDistanceWeight());
        Logger.recordOutput("Vision/Tuning/RotationDistanceWeight", getRotationDistanceWeight());
        Logger.recordOutput("Vision/Tuning/MaxSingleTagDistance", getMaxSingleTagDistance());
        Logger.recordOutput("Vision/Tuning/MaxMultiTagDistance", getMaxMultiTagDistance());
        Logger.recordOutput("Vision/Tuning/MaxSingleTagPoseDeltaMeters", getMaxSingleTagPoseDeltaMeters());
        Logger.recordOutput("Vision/Tuning/MaxMultiTagPoseDeltaMeters", getMaxMultiTagPoseDeltaMeters());
    }

    /**
     * Runs ONE camera frame through the quality checks and, if it survives,
     * packages it up as a VisionMeasurement.
     *
     * CHANGE #2 lives here: ambiguity and distance no longer act like walls.
     * They act like a dimmer switch — the worse the measurement, the less the
     * estimator believes it. Only the "teleport guard" (CHANGE #2b) still
     * rejects outright, because a measurement that says we suddenly jumped a
     * metre isn't "worse data", it's wrong data.
     */
    private Optional<VisionMeasurement> evaluateFrame(CameraModule module, PhotonPipelineResult result) {
        // No tags in this frame? Nothing to add. (Not counted as a "rejection" —
        // there was simply nothing to see. Counting these made the old reject
        // counter look alarming during perfectly normal empty frames.)
        if (!result.hasTargets()) {
            module.lastRejectReason = "No AprilTags in frame";
            return Optional.empty();
        }

        int tagCount = result.getTargets().size();
        boolean isMultiTag = tagCount > 1;
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        // ---- CHANGE #2a: ambiguity trust ramp (single-tag only) ----
        // Ambiguity answers: "could this tag plausibly be tilted the other way?"
        // 0 = confident, 1 = a coin flip. Multi-tag solves don't have this problem
        // (several tags can't all flip the same way), so this only gates single-tag.
        double ambiguityFactor = 1.0;
        if (!isMultiTag) {
            double ambiguity = bestTarget.getPoseAmbiguity();
            if (ambiguity > VisionConstants.kMaxAmbiguity) {
                if (ambiguity >= VisionConstants.kAmbiguityRejectLimit) {
                    // Truly ambiguous — even believing it at 1/6 weight would hurt.
                    module.lastRejectReason = String.format(
                        "Ambiguity %.3f >= reject limit %.3f", ambiguity, VisionConstants.kAmbiguityRejectLimit);
                    module.rejectedFrameCount++;
                    return Optional.empty();
                }
                // In the "grey zone": keep it, but shrink its vote. At the top of
                // the grey zone we're down to 1/kAmbiguityMaxPenalty of the trust.
                double t = (ambiguity - VisionConstants.kMaxAmbiguity)
                    / (VisionConstants.kAmbiguityRejectLimit - VisionConstants.kMaxAmbiguity);
                ambiguityFactor = 1.0 + t * (VisionConstants.kAmbiguityMaxPenalty - 1.0);
            }
        }

        // Ask PhotonVision for a robot pose from this frame. Prefer the
        // coprocessor's multi-tag solve (it runs on the Pi/etc. and is stronger);
        // fall back to the lowest-ambiguity single tag solved on the RoboRIO.
        // NOTE: both paths assume the coprocessor and this code agree on the
        // field layout — see CHANGE #4 at the top of the file.
        Optional<EstimatedRobotPose> visionEst = module.poseEstimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = module.poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (visionEst.isEmpty()) {
            module.lastRejectReason = "PhotonPoseEstimator returned no pose";
            module.rejectedFrameCount++;
            return Optional.empty();
        }

        // ---- CHANGE #2a (continued): distance trust ramp ----
        double averageTagDistanceMeters = getAverageTagDistanceMeters(result);
        double maxDistanceMeters = isMultiTag ? getMaxMultiTagDistance() : getMaxSingleTagDistance();
        double distanceFactor = 1.0;
        if (averageTagDistanceMeters > maxDistanceMeters) {
            double overRatio = averageTagDistanceMeters / maxDistanceMeters;
            if (overRatio > VisionConstants.kDistanceRejectMultiplier) {
                // Farther than 1.5x the trusted range: error grows roughly with
                // distance squared, so out here the numbers are mostly noise.
                module.lastRejectReason = String.format(
                    "Tag distance %.2fm > hard limit %.2fm",
                    averageTagDistanceMeters, maxDistanceMeters * VisionConstants.kDistanceRejectMultiplier);
                module.rejectedFrameCount++;
                return Optional.empty();
            }
            // Slightly past the trusted range: quadratic penalty, still usable.
            distanceFactor = overRatio * overRatio;
        }

        // ---- CHANGE #2b: the teleport guard (still a hard wall, on purpose) ----
        // If the camera claims we jumped farther than a metre since the last
        // estimate, either it saw a ghost or something is badly wrong. Believing
        // it "a bit" makes no sense, so we drop it and count it.
        Pose2d estimatedPose = visionEst.get().estimatedPose.toPose2d();
        Pose2d currentPose = drivetrain.getPose();
        double translationDeltaMeters = estimatedPose.getTranslation().getDistance(currentPose.getTranslation());
        double maxTranslationDeltaMeters = isMultiTag
            ? getMaxMultiTagPoseDeltaMeters()
            : getMaxSingleTagPoseDeltaMeters();

        SmartDashboard.putNumber("Vision/" + module.name + "/PoseDeltaMeters", translationDeltaMeters);
        Logger.recordOutput("Vision/" + module.name + "/PoseDeltaMeters", translationDeltaMeters);

        if (translationDeltaMeters > maxTranslationDeltaMeters) {
            module.lastRejectReason = String.format(
                "Pose delta %.2fm > %.2fm (teleport guard)", translationDeltaMeters, maxTranslationDeltaMeters);
            module.rejectedFrameCount++;
            return Optional.empty();
        }

        // Build the final uncertainty numbers (this is where CHANGE #3 happens).
        Matrix<N3, N1> estStdDevs =
            getEstimationStdDevs(result, averageTagDistanceMeters, ambiguityFactor * distanceFactor);

        return Optional.of(new VisionMeasurement(
            visionEst.get(),
            estStdDevs,
            tagCount,
            averageTagDistanceMeters
        ));
    }

    /**
     * Computes the standard deviations ("how much do we trust this") for a frame.
     *
     * A standard deviation is a trust knob: SMALL number = "believe me strongly",
     * BIG number = "eh, take with a grain of salt", INFINITY = "ignore this part".
     *
     * CHANGE #3: for SINGLE-tag frames we now report INFINITE heading uncertainty.
     * The pose estimator therefore takes the X/Y correction but never lets a lone
     * distant tag rotate the robot's belief. The NavX gyro owns heading — it's far
     * better at it. Multi-tag frames still contribute heading because seeing
     * several tags at once pins rotation down reliably.
     *
     * The trust factors from CHANGE #2 multiply in, so a sketchy-but-usable frame
     * gets proportionally less influence instead of being thrown away.
     */
    private Matrix<N3, N1> getEstimationStdDevs(
        PhotonPipelineResult result, double averageTagDistanceMeters, double distrustFactor) {

        double xyStdDev;
        double thetaStdDev;

        // =================================================================
        // CHANGE #5 (heading authority): THE GYRO IS THE ONLY HEADING SOURCE.
        //
        // Team decision: vision never corrects the robot's HEADING — not from
        // one tag, not even from several. Vision corrects LOCATION (X/Y) only;
        // the gyroscope owns rotation completely.
        //
        // Why this is the right call for OUR robot right now:
        //   * A camera-derived heading depends on exact camera mounting angles
        //     and an exactly-correct field layout. Both are still being
        //     verified (see REVIEW.md A1), so vision heading is our least
        //     trustworthy signal — and heading errors rotate the WHOLE pose,
        //     which also corrupts where X/Y corrections land.
        //   * The gyro measures rotation directly and smoothly. Its slow drift
        //     over one match (< ~1 degree) is far smaller than what a flickery
        //     tag was adding.
        //   * Mechanical Advantage (6328) allows vision rotation ONLY for
        //     multi-tag solves. We go one step more conservative until the
        //     layout audit and repeatability testing pass. To restore
        //     MA-style multi-tag heading later, set kMultiTagHeadingEnabled
        //     to true in VisionConstants — nothing else changes.
        //
        // Infinite standard deviation = "estimator, please ignore this part".
        // =================================================================
        int numTags = result.getTargets().size();
        if (numTags > 1 && VisionConstants.kMultiTagHeadingEnabled) {
            // Multi-tag AND heading enabled: strong on position AND rotation.
            xyStdDev = getMultiTagXYStdDev();
            thetaStdDev = getMultiTagThetaStdDev();
        } else {
            // Position only. Infinite theta = "don't touch heading".
            xyStdDev = getSingleTagXYStdDev();
            thetaStdDev = Double.POSITIVE_INFINITY;
        }

        // Farther away = pixel errors turn into bigger real-world errors.
        // (Squaring the distance for XY mirrors how camera error actually grows.)
        xyStdDev *= 1 + averageTagDistanceMeters * averageTagDistanceMeters * getDistanceWeight();
        thetaStdDev *= 1 + averageTagDistanceMeters * getRotationDistanceWeight();

        // Apply the soft-penalty factor from the ambiguity/distance ramps.
        xyStdDev *= distrustFactor;

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    private void publishTuningDefaults() {
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MaxAmbiguity", VisionConstants.kMaxAmbiguity);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "SingleTagXYStdDev", VisionConstants.kSingleTagXYStdDev);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MultiTagXYStdDev", VisionConstants.kMultiTagXYStdDev);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MultiTagThetaStdDev", VisionConstants.kMultiTagThetaStdDev);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "DistanceWeight", VisionConstants.kDistanceWeight);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "RotationDistanceWeight", VisionConstants.kRotationDistanceWeight);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MaxSingleTagDistance", VisionConstants.kMaxSingleTagDistance);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MaxMultiTagDistance", VisionConstants.kMaxMultiTagDistance);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MaxSingleTagPoseDeltaMeters", VisionConstants.kMaxSingleTagPoseDeltaMeters);
        SmartDashboard.putNumber(TUNING_KEY_PREFIX + "MaxMultiTagPoseDeltaMeters", VisionConstants.kMaxMultiTagPoseDeltaMeters);
    }

    private double getTuningValue(String keySuffix, double defaultValue) {
        return SmartDashboard.getNumber(TUNING_KEY_PREFIX + keySuffix, defaultValue);
    }

    private double getMaxAmbiguity() {
        return getTuningValue("MaxAmbiguity", VisionConstants.kMaxAmbiguity);
    }

    private double getSingleTagXYStdDev() {
        return getTuningValue("SingleTagXYStdDev", VisionConstants.kSingleTagXYStdDev);
    }

    private double getMultiTagXYStdDev() {
        return getTuningValue("MultiTagXYStdDev", VisionConstants.kMultiTagXYStdDev);
    }

    private double getMultiTagThetaStdDev() {
        return getTuningValue("MultiTagThetaStdDev", VisionConstants.kMultiTagThetaStdDev);
    }

    private double getDistanceWeight() {
        return getTuningValue("DistanceWeight", VisionConstants.kDistanceWeight);
    }

    private double getRotationDistanceWeight() {
        return getTuningValue("RotationDistanceWeight", VisionConstants.kRotationDistanceWeight);
    }

    private double getMaxSingleTagDistance() {
        return getTuningValue("MaxSingleTagDistance", VisionConstants.kMaxSingleTagDistance);
    }

    private double getMaxMultiTagDistance() {
        return getTuningValue("MaxMultiTagDistance", VisionConstants.kMaxMultiTagDistance);
    }

    private double getMaxSingleTagPoseDeltaMeters() {
        return getTuningValue("MaxSingleTagPoseDeltaMeters", VisionConstants.kMaxSingleTagPoseDeltaMeters);
    }

    private double getMaxMultiTagPoseDeltaMeters() {
        return getTuningValue("MaxMultiTagPoseDeltaMeters", VisionConstants.kMaxMultiTagPoseDeltaMeters);
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
     */
    public List<CameraModule> getCameraModules() {
        return cameraModules;
    }

    /**
     * Gets the number of active cameras.
     */
    public int getActiveCameraCount() {
        return cameraModules.size();
    }

    /**
     * Gets the AprilTag field layout.
     */
    public AprilTagFieldLayout getFieldLayout() {
        return aprilTagFieldLayout;
    }

    public List<Pose3d> getDetectedTagFieldPoses() {
        List<Pose3d> detectedTagFieldPoses = new ArrayList<>();
        Set<Integer> loggedTagIds = new LinkedHashSet<>();

        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.lastResult;
            if (result == null || !result.hasTargets()) {
                continue;
            }

            for (PhotonTrackedTarget target : result.getTargets()) {
                int tagId = target.getFiducialId();
                if (!loggedTagIds.add(tagId)) {
                    continue;
                }

                aprilTagFieldLayout.getTagPose(tagId).ifPresent(detectedTagFieldPoses::add);
            }
        }

        return detectedTagFieldPoses;
    }

    public List<Pose3d> getCameraFieldPoses() {
        List<Pose3d> cameraFieldPoses = new ArrayList<>();
        Pose3d robotPose = new Pose3d(drivetrain.getPose());

        for (CameraModule module : cameraModules) {
            cameraFieldPoses.add(robotPose.transformBy(VisionConstants.kRobotToCams[module.index]));
        }

        return cameraFieldPoses;
    }

    /**
     * Gets a list of all currently visible AprilTag IDs from all cameras.
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
     */
    public List<AprilTagInfo> getDetailedAprilTagInfo() {
        List<AprilTagInfo> tagInfoList = new ArrayList<>();

        for (CameraModule module : cameraModules) {
            PhotonPipelineResult result = module.lastResult;
            if (result != null && result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    double ambiguity = target.getPoseAmbiguity();
                    // CHANGE #2 note for dashboards: "usedForPoseUpdate" now means
                    // "not hard-rejected". Grey-zone frames ARE used, just trusted less.
                    boolean usedForUpdate = !(ambiguity >= VisionConstants.kAmbiguityRejectLimit);

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
     */
    public TagNavigationInfo getTagNavigationInfo(int tagId) {
        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(tagId);

        if (!tagPoseOptional.isPresent()) {
            return new TagNavigationInfo(tagId, false, 0.0, 0.0);
        }

        Pose2d tagPose = tagPoseOptional.get().toPose2d();
        Pose2d robotPose = drivetrain.getPose();

        double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());

        double deltaX = tagPose.getX() - robotPose.getX();
        double deltaY = tagPose.getY() - robotPose.getY();
        double heading = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return new TagNavigationInfo(tagId, true, distance, heading);
    }
}
