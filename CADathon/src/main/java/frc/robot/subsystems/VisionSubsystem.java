package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontRight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightObject.Location;
import com.ctre.phoenix6.Utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * VisionSubsystem using Limelights with fusion, singleton pattern, and drivetrain integration.
 */
public class VisionSubsystem extends SubsystemBase {

    // -------------------------
    // Singleton instance
    // -------------------------
    private static VisionSubsystem instance = null;
    public static synchronized VisionSubsystem getInstance(CommandSwerveDrivetrain drivetrain) {
        if (instance == null) {
            instance = new VisionSubsystem(drivetrain);
        }
        return instance;
    }

    // -------------------------
    // Core subsystem fields
    // -------------------------
    private final CommandSwerveDrivetrain drivetrain;

    private volatile Pose2d lastVisionPose = null;
    private volatile double lastVisionTimestamp = 0.0;

    private double visionStdDevX = 0.7;
    private double visionStdDevY = 0.7;
    private double visionStdDevTheta = 9999999.0;

    private double maxTranslationDiff = 0.5; // meters
    private double maxRotationDiff = Math.toRadians(10); // radians

    // Placeholder shooter linear model
    private double fallbackRPMPerMeter = 1000.0;
    private double fallbackRPMIntercept = 1500.0;
    private double fallbackHoodPerMeter = 10.0;
    private double fallbackHoodIntercept = 5.0;

    // Limelight objects
    private final List<LimelightObject> limelightList = new ArrayList<>();
    private LimelightObject limelightFL;
    private LimelightObject limelightFR;

    // Logging
    private volatile double lastFLConfidence = 0.0;
    private volatile double lastFRConfidence = 0.0;
    private volatile Pose2d lastFusedPose = null;

    // -------------------------
    // Constructor (private for singleton)
    // -------------------------
    private VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        configureLimelights();

        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(visionStdDevX, visionStdDevY, visionStdDevTheta));
    }

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

    @Override
    public void periodic() {
        updateVision();
        publishLogs();
    }

    // -------------------------
    // Vision update + fusion
    // -------------------------
    private void updateVision() {
        // Update Limelight robot orientation (use drivetrain yaw)
        double robotYaw = drivetrain.getPigeon2().getYaw().getValueAsDouble() - RobotContainer.pigeonOffset;

        LimelightHelpers.SetRobotOrientation(limelightFL.getName(), robotYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate peFL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightFL.getName());

        LimelightHelpers.SetRobotOrientation(limelightFR.getName(), robotYaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate peFR = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightFR.getName());

        VisionMeasurement vmFL = createVisionMeasurement(peFL, limelightFL.getName());
        VisionMeasurement vmFR = createVisionMeasurement(peFR, limelightFR.getName());

        lastFLConfidence = vmFL.confidence;
        lastFRConfidence = vmFR.confidence;

        Optional<Pose2d> fusedPose = fuseVision(vmFL, vmFR);

        fusedPose.ifPresent(pose -> {
            lastVisionPose = pose;
            lastFusedPose = pose;
            lastVisionTimestamp = Math.max(vmFL.timestamp, vmFR.timestamp);
            drivetrain.addVisionMeasurement(pose, lastVisionTimestamp);
        });
    }

    private VisionMeasurement createVisionMeasurement(LimelightHelpers.PoseEstimate pe, String limelightName) {
        if (pe == null || pe.pose == null) return VisionMeasurement.invalid();

        double ambiguity = computeAmbiguity(limelightName);
        double timestamp = Utils.fpgaToCurrentTime(pe.timestampSeconds);
        Pose2d pose = (Pose2d) pe.pose;
        int tagCount = pe.tagCount;
        boolean valid = tagCount > 0 && ambiguity < 0.35;
        double tagFactor = Math.min(tagCount / 2.0, 1.0);
        double confidence = (1.0 - ambiguity) * 0.7 + tagFactor * 0.3;

        return new VisionMeasurement(pose, confidence, timestamp, valid);
    }

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

    private Optional<Pose2d> fuseVision(VisionMeasurement vmFL, VisionMeasurement vmFR) {
        boolean flGood = vmFL.isValid();
        boolean frGood = vmFR.isValid();

        if (flGood && !frGood) return Optional.of(vmFL.pose());
        if (!flGood && frGood) return Optional.of(vmFR.pose());
        if (!flGood && !frGood) return Optional.empty();

        double translationDiff = vmFL.pose().getTranslation().getDistance(vmFR.pose().getTranslation());
        double rotationDiff = Math.abs(vmFL.pose().getRotation().minus(vmFR.pose().getRotation()).getRadians());

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
            return vmFL.confidence >= vmFR.confidence ? Optional.of(vmFL.pose()) : Optional.of(vmFR.pose());
        }
    }

    // -------------------------
    // Logging
    // -------------------------
    private void publishLogs() {
        System.out.printf("[Vision] FusedPose: %s | FLConf: %.2f | FRConf: %.2f%n",
                lastFusedPose, lastFLConfidence, lastFRConfidence);
    }

    public Pose2d getLastFusedPose() { return lastFusedPose; }
    public double getLastFLConfidence() { return lastFLConfidence; }
    public double getLastFRConfidence() { return lastFRConfidence; }

    // -------------------------
    // Public API
    // -------------------------
    public Optional<Pose2d> getLastVisionPose() { return Optional.ofNullable(lastVisionPose); }

    public double getDistanceToTarget(Pose2d target) {
        if (lastVisionPose == null) return Double.POSITIVE_INFINITY;
        return lastVisionPose.getTranslation().getDistance(target.getTranslation());
    }

    public Transform2d getRobotToTarget(Pose2d target) {
        if (lastVisionPose == null) return null;
        Translation2d deltaField = target.getTranslation().minus(lastVisionPose.getTranslation());
        Translation2d translationRobotFrame = deltaField.rotateBy(lastVisionPose.getRotation().unaryMinus());
        Rotation2d rotationRobotToTarget = target.getRotation().minus(lastVisionPose.getRotation());
        return new Transform2d(translationRobotFrame, rotationRobotToTarget);
    }

    public ShooterSettings recommendShooterForTarget(Pose2d target) {
        double dist = getDistanceToTarget(target);
        if (!Double.isFinite(dist)) return new ShooterSettings(fallbackHoodIntercept, fallbackRPMIntercept);
        double rpm = fallbackRPMPerMeter * dist + fallbackRPMIntercept;
        double hood = fallbackHoodPerMeter * dist + fallbackHoodIntercept;
        return new ShooterSettings(hood, rpm);
    }

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
        @Override public String toString() { return String.format("Hood=%.2fdeg, RPM=%.0f", hoodDegrees, wheelRPM); }
    }

    private record VisionMeasurement(Pose2d pose, double confidence, double timestamp, boolean valid) {
        static VisionMeasurement invalid() { return new VisionMeasurement(null, 0.0, 0.0, false); }
        boolean isValid() { return valid && pose != null; }
    }
}
