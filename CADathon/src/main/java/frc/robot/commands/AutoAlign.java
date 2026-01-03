package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.path.RotationTarget;

import javax.sound.midi.ControllerEventListener;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.util.BezierCurve;

public class AutoAlign extends Command{

    private CommandSwerveDrivetrain drive;
    private VisionSubsystem vision = RobotContainer.vision;

    private CommandXboxController controller;

    private CoordinationSubsystem coordSub = CoordinationSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                                                                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private Pose2d currentPose;
    private Pose2d targetPose;
    private Translation2d targetRotation;
    
    private PIDController rotationPID = new PIDController(6, 0, 0);
    private PIDController xPID = new PIDController(5, 0, 0);
    private PIDController yPID = new PIDController(5, 0, 0);
    
    private boolean startedShoot = false;

    public AutoAlign(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, Pose2d target) {
        drive = drivetrain;
        this.controller = controller;

        targetPose = target;
        vision = RobotContainer.vision;
        
        currentPose = drive.getState().Pose;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        targetRotation = vision.getRotationTarget();
        currentPose = vision.getLastVisionPose();
        startedShoot = false;
        rotationPID.setSetpoint(calculateRotationAngleTarget());
    }

    @Override
    public void execute() {
        // currentPose = drive.getState().Pose;
        currentPose = vision.getLastVisionPose();

        rotationPID.setSetpoint(calculateRotationAngleTarget());
        if(atSetpoint(0.1, 0.1)) {
            controller.setRumble(RumbleType.kBothRumble, 0.5);
            if(!startedShoot) {
                coordSub.setState(AbsoluteStates.PREPARING_FOR_SHOT_OVERRIDE);
                shooter.setShooterAngleSetpoint(11.5068);
                shooter.setVelocityOverrideSetpoint(32);
                startedShoot = true;
            }
        }
        else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
        if(!startedShoot) {
            xPID.setSetpoint(targetPose.getX());
            yPID.setSetpoint(targetPose.getY());
            double powerRotation = rotationPID.calculate(currentPose.getRotation().getRadians());
            double powerX = MathUtil.clamp(xPID.calculate(currentPose.getX()), -1.5, 1.5);
            double powerY = MathUtil.clamp(yPID.calculate(currentPose.getY()), -1.5, 1.5);

            powerX += .05*Math.signum(powerX);
            powerY += .05*Math.signum(powerY);


            SwerveRequest request = driveRequest
            .withVelocityX(powerX)
            .withVelocityY(powerY)
            .withRotationalRate(powerRotation);
            drive.setControl(request);
        }
        else {

            double powerRotation = rotationPID.calculate(currentPose.getRotation().getRadians());
            SwerveRequest request = driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(powerRotation);
            drive.setControl(request);
        }
        publishLogs();


    }


    public void publishLogs() {
        Logger.recordOutput("HeroHeist/Vision/CurrentPose", currentPose);
        Logger.recordOutput("HeroHeist/Vision/targetPose", targetPose);
    }

    public boolean atSetpoint(double translationTolerance, double rotationalTolerance) {
        return Math.abs(xPID.getSetpoint() - currentPose.getX()) < translationTolerance && Math.abs(yPID.getSetpoint() - currentPose.getY()) < translationTolerance && Math.abs(rotationPID.getError()) < rotationalTolerance;
    }
    
    public double calculateRotationAngleTarget() {
        double xDiff = targetRotation.getX() - currentPose.getX();
        double yDiff = targetRotation.getY() - currentPose.getY();
        double rotationTarget = Math.atan(yDiff / xDiff);

        if(currentPose.getX() > targetRotation.getX()) {
            rotationTarget += Math.PI;
        }

        return rotationTarget;

    }

    public void end(boolean interrupted) {
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drive.setControl(stop);
        controller.setRumble(RumbleType.kBothRumble, 0);
        coordSub.setState(AbsoluteStates.STORING);
    }
}
