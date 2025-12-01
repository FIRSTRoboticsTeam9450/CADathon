package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontRight;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightObject.Location;
import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

/**
 * VisionSubsystem using Limelights with:
 *  - Fusion of multiple cameras
 *  - Singleton pattern
 *  - Integration with drivetrain for pose estimation
 *
 * This subsystem reads Limelight data, fuses multiple camera poses,
 * and provides utility functions like distance to target and shooter recommendations.
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------
    // Singleton instance
    // -------------------------
    private static VisionSubsystem instance = null;

    /**
     * Singleton accessor method.
     * Ensures only one instance exists and is globally accessible.
     *
     * @param drivetrain Drivetrain instance for updating pose with vision
     * @return VisionSubsystem instance
     */
    public static synchronized VisionSubsystem getInstance(CommandSwerveDrivetrain drivetrain) {
        if (instance == null) {
            instance = new VisionSubsystem(drivetrain);
        }
        return instance;
    }

    public static synchronized VisionSubsystem getInstance() {
        return getInstance(drivetrain);
    }

    // -------------------------
    // Core subsystem fields
    // -------------------------
    private static CommandSwerveDrivetrain drivetrain; // Reference to drivetrain for pose updates

    private volatile Pose2d lastVisionPose = null;  // Last pose estimated from fused vision
    private volatile double lastVisionTimestamp = 0.0; // Timestamp of last vision update

    // Standard deviations for vision measurements (for filtering/fusion)
    private double visionStdDevX = 0.7;
    private double visionStdDevY = 0.7;
    private double visionStdDevTheta = 9999999.0;

    // Maximum allowed differences for fusion; if exceeded, fallback to higher confidence camera
    private double maxTranslationDiff = 0.5; // meters
    private double maxRotationDiff = Math.toRadians(10); // radians

    // Placeholder linear model for shooter if vision is unavailable
    private double fallbackRPMPerMeter = 1000.0;
    private double fallbackRPMIntercept = 1500.0;
    private double fallbackHoodPerMeter = 10.0;
    private double fallbackHoodIntercept = 5.0;

    // Limelight camera objects
    private final List<LimelightObject> limelightList = new ArrayList<>();
    private LimelightObject limelightFL; // Front-left camera
    private LimelightObject limelightFR; // Front-right camera

    // Logging
    private volatile double lastFLConfidence = 0.0; // Last confidence reading from front-left camera
    private volatile double lastFRConfidence = 0.0; // Last confidence reading from front-right camera
    private volatile Pose2d lastFusedPose = null;   // Last fused pose from vision
    private volatile Pose2d lastFLPose = null;      // Last known pose from only FL data
    private volatile Pose2d lastFRPose = null;      // Last known pose from only FR data
    private volatile Pose2d lastTargetPose = null;


    // -------------------------
    // Constructor (private for singleton)
    // -------------------------
    @SuppressWarnings("static-access")
    private VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Configure camera offsets and add to internal list
        configureLimelights();

        // Initialize drivetrain vision standard deviations
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
    }

    /**
     * Initializes LimelightObject instances with physical offsets and names.
     * Ensures that all camera poses are correctly transformed to robot frame.
     */
    private void configureLimelights() {
        limelightFL = new LimelightObject(FrontLeft.NAME)
                .withOffsetX(FrontLeft.LOCATION_X)
                .withOffsetY(FrontLeft.LOCATION_Y)
                .withOffsetZ(FrontLeft.LOCATION_Z)
                .withOffsetPitch(FrontLeft.PITCH)
                .withOffsetRoll(FrontLeft.ROLL)
                .withOffsetYaw(FrontLeft.YAW);
        limelightFL.location = Location.FRONT_LEFT;

        limelightFR = new LimelightObject(FrontRight.NAME)
                .withOffsetX(FrontRight.LOCATION_X)
                .withOffsetY(FrontRight.LOCATION_Y)
                .withOffsetZ(FrontRight.LOCATION_Z)
                .withOffsetPitch(FrontRight.PITCH)
                .withOffsetRoll(FrontRight.ROLL)
                .withOffsetYaw(FrontRight.YAW);
        limelightFR.location = Location.FRONT_RIGHT;

        limelightList.add(limelightFL);
        limelightList.add(limelightFR);
    }

    /**
     * Periodic method called every ~20ms by the scheduler.
     * Updates vision data and logs key metrics.
     */
    @Override
    public void periodic() {
        updateVision();  // Compute fused vision pose
        publishLogs();   // Print/log fusion confidence and pose
    }

    // -------------------------
    // Vision update + fusion
    // -------------------------

    /**
     * Updates pose estimates from both Limelight cameras and fuses them.
     * Sends fused pose to drivetrain for odometry correction.
     */
    private void updateVision() {
        // Get robot yaw from drivetrain's Pigeon2 sensor, applying offset
        double robotYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble() - RobotContainer.pigeonOffset;

        // Update each Limelight with current robot yaw
        LimelightHelpers.SetRobotOrientation(limelightFL.getName(), robotYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate peFL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightFL.getName());
        lastFLPose = peFL.pose;

        LimelightHelpers.SetRobotOrientation(limelightFR.getName(), robotYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate peFR = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightFR.getName());
        lastFRPose = peFR.pose;

        // Convert raw Limelight data into VisionMeasurement objects
        VisionMeasurement vmFL = createVisionMeasurement(peFL, limelightFL.getName());
        VisionMeasurement vmFR = createVisionMeasurement(peFR, limelightFR.getName());

        // Update logging confidence values
        lastFLConfidence = vmFL.confidence;
        lastFRConfidence = vmFR.confidence;

        // Fuse camera measurements
        Optional<Pose2d> fusedPose = fuseVision(vmFL, vmFR);

        // If valid, update lastVisionPose and send to drivetrain
        fusedPose.ifPresent(pose -> {
            lastVisionPose = pose;
            lastFusedPose = pose;
            lastVisionTimestamp = Math.max(vmFL.timestamp, vmFR.timestamp);
            drivetrain.addVisionMeasurement(pose, lastVisionTimestamp);
        });
    }

    /**
     * Converts a Limelight PoseEstimate into a VisionMeasurement.
     * Includes confidence calculation and validity check.
     */
    private VisionMeasurement createVisionMeasurement(LimelightHelpers.PoseEstimate pe, String limelightName) {
        if (pe == null || pe.pose == null) return VisionMeasurement.invalid();

        double ambiguity = computeAmbiguity(limelightName);
        double timestamp = Utils.fpgaToCurrentTime(pe.timestampSeconds);
        Pose2d pose = (Pose2d) pe.pose;
        int tagCount = pe.tagCount;

        // Consider measurement valid if at least 1 tag and ambiguity is low
        boolean valid = tagCount > 0 && ambiguity < 0.35;

        // Confidence weighted: 70% ambiguity, 30% tag count
        double tagFactor = Math.min(tagCount / 2.0, 1.0);
        double confidence = (1.0 - ambiguity) * 0.7 + tagFactor * 0.3;

        return new VisionMeasurement(pose, confidence, timestamp, valid);
    }

    /**
     * Computes the best (lowest) ambiguity among all fiducials seen by a Limelight.
     * @Returns 1.0 if no fiducials detected (completely ambiguous).
     */
    private double computeAmbiguity(String limelightName) {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        if (fiducials == null || fiducials.length == 0) return 1.0;

        double best = 1.0;
        for (LimelightHelpers.RawFiducial f : fiducials) {
            if (f == null) continue;
            best = Math.min(best, f.ambiguity);
        }
        return best;
    }

    /**
     * Fuses two VisionMeasurements into a single Pose2d.
     * If both measurements are valid and within allowed thresholds, returns confidence-weighted fusion.
     * Otherwise, returns the measurement with higher confidence.
     */
    private Optional<Pose2d> fuseVision(VisionMeasurement vmFL, VisionMeasurement vmFR) {
        boolean flGood = vmFL.isValid();
        boolean frGood = vmFR.isValid();

        if (flGood && !frGood) return Optional.of(vmFL.pose());
        if (!flGood && frGood) return Optional.of(vmFR.pose());
        if (!flGood && !frGood) return Optional.empty();

        // Compute translation and rotation differences
        double translationDiff = vmFL.pose().getTranslation().getDistance(vmFR.pose().getTranslation());
        double rotationDiff = Math.abs(vmFL.pose().getRotation().minus(vmFR.pose().getRotation()).getRadians());

        // If cameras largely agree, perform confidence-weighted fusion
        if (translationDiff < maxTranslationDiff && rotationDiff < maxRotationDiff) {
            double totalConfidence = vmFL.confidence + vmFR.confidence;
            double wFL = vmFL.confidence / totalConfidence;
            double wFR = vmFR.confidence / totalConfidence;

            Translation2d fusedTranslation = new Translation2d(
                    vmFL.pose().getX() * wFL + vmFR.pose().getX() * wFR,
                    vmFL.pose().getY() * wFL + vmFR.pose().getY() * wFR
            );
            double fusedRotation = vmFL.pose().getRotation().getRadians() * wFL +
                    vmFR.pose().getRotation().getRadians() * wFR;

            return Optional.of(new Pose2d(fusedTranslation, new Rotation2d(fusedRotation)));
        } else {
            // If disagreement too large, fallback to higher-confidence camera
            return vmFL.confidence >= vmFR.confidence ? Optional.of(vmFL.pose()) : Optional.of(vmFR.pose());
        }
    }

    // -------------------------
    // Logging
    // -------------------------
    private void publishLogs() {
        Logger.recordOutput("HeroHeist/Vision/Total/Last Vision Pose", lastVisionPose);
        Logger.recordOutput("HeroHeist/Vision/Total/Last Fused Pose", lastFusedPose);
        Logger.recordOutput("HeroHeist/Vision/Total/Last Target Pose", lastTargetPose);

        Logger.recordOutput("HeroHeist/Vision/FL/Last FL-Data only Pose", lastFLPose);
        Logger.recordOutput("HeroHeist/Vision/FL/Last FL Confidence", lastFLConfidence);

        Logger.recordOutput("HeroHeist/Vision/FR/Last FR-Data only Pose", lastFRPose);
        Logger.recordOutput("HeroHeist/Vision/FR/Last FR Confidence", lastFRConfidence);
    }

    // -------------------------
    // Public API
    // -------------------------
    public Pose2d getLastFusedPose() { return lastFusedPose; }
    public double getLastFLConfidence() { return lastFLConfidence; }
    public double getLastFRConfidence() { return lastFRConfidence; }
    public Optional<Pose2d> getLastVisionPose() { return Optional.ofNullable(lastVisionPose); }

    /**
     * Computes the distance from robot to a target pose.
     */
    public double getDistanceToTarget(Pose2d target) {
        if (lastVisionPose == null) return Double.POSITIVE_INFINITY;
        lastTargetPose = target;
        return lastVisionPose.getTranslation().getDistance(target.getTranslation());
    }

    public targetData getTargetData(String town) {
        if(town.equals("uptown") ) {

        }
        else if(town.equals("downtown")) {

        }
        else if(town.equals("district")) {

        }
    }
    
    /**
     * Computes the robot-to-target transform in robot frame.
     */
    public Transform2d getRobotToTarget(Pose2d target) {
        if (lastVisionPose == null) return null;

        lastTargetPose = target;

        Translation2d deltaField = target.getTranslation().minus(lastVisionPose.getTranslation());
        Translation2d translationRobotFrame = deltaField.rotateBy(lastVisionPose.getRotation().unaryMinus());
        Rotation2d rotationRobotToTarget = target.getRotation().minus(lastVisionPose.getRotation());

        return new Transform2d(translationRobotFrame, rotationRobotToTarget);
    }

    /**
     * Provides shooter hood angle and wheel RPM recommendations based on distance.
     * Uses fallback linear model if vision unavailable.
     * REPLACE WITH CALCULATIONS ON WHITEBOARD WHEN CAN GET THEM
     */
    public ShooterSettings recommendShooterForTarget(Pose2d target, String town) {

        lastTargetPose = target;
        
        targetData targetData = getTargetData(town);
        double dist = getDistanceToTarget(target);
        double targetHeight = targetData.targetHeight;
        double targetAngle = targetData.targetAngle;
        double distOffset = targetData.distanceOffset;

        if (!Double.isFinite(dist)) return new ShooterSettings(fallbackHoodIntercept, fallbackRPMIntercept);
        double hood = Math.atan(2 * (targetHeight - Constants.ShooterConstants.SHOOTER_HEIGHT)/dist - Math.tan(targetAngle));
        double rpm = 1/Math.cos(targetAngle) * Math.sqrt(9.81 * dist/ Math.abs(Math.tan(targetAngle) - Math.tan(hood)));
        // double rpm = fallbackRPMPerMeter * dist + fallbackRPMIntercept;
        // double hood = fallbackHoodPerMeter * dist + fallbackHoodIntercept;

        return new ShooterSettings(hood, rpm);
    }

    /**
     * Updates vision measurement standard deviations for fusion/filtering.
     */
    public void setVisionMeasurementStdDevs(double xMeters, double yMeters, double thetaRadians) {
        this.visionStdDevX = xMeters;
        this.visionStdDevY = yMeters;
        this.visionStdDevTheta = thetaRadians;

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
    }

    public void setFallbackShooterLinear(double rpmPerMeter, double rpmIntercept, double hoodPerMeter, double hoodIntercept) {
        this.fallbackRPMPerMeter = rpmPerMeter;
        this.fallbackRPMIntercept = rpmIntercept;
        this.fallbackHoodPerMeter = hoodPerMeter;
        this.fallbackHoodIntercept = hoodIntercept;
    }

    public void setMaxDiffForFusion(double translationMeters, double rotationRadians) {
        this.maxTranslationDiff = translationMeters;
        this.maxRotationDiff = rotationRadians;
    }

    @SuppressWarnings("static-access")
    public void setDrivetrainInstance(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // -------------------------
    // Inner classes
    // -------------------------
    public static class ShooterSettings {
        public final double hoodDegrees;
        public final double wheelRPM;

        public ShooterSettings(double hoodDegrees, double wheelRPM) {
            this.hoodDegrees = hoodDegrees;
            this.wheelRPM = wheelRPM;
        }

        @Override
        public String toString() {
            return String.format("Hood=%.2fdeg, RPM=%.0f", hoodDegrees, wheelRPM);
        }
    }

    public static class targetData {
        public final double targetAngle;
        public final double targetHeight;
        public final double distanceOffset;
    }

    /**
     * Wrapper class for vision measurements.
     * Contains pose, confidence, timestamp, and validity flag.
     */
    private record VisionMeasurement(Pose2d pose, double confidence, double timestamp, boolean valid) {
        static VisionMeasurement invalid() {
            return new VisionMeasurement(null, 0.0, 0.0, false);
        }

        boolean isValid() {
            return valid && pose != null;
        }
    }
}
