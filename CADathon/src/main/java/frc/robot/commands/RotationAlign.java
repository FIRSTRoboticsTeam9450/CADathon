package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BezierCurve;

public class RotationAlign extends Command{

    private CommandSwerveDrivetrain drive;
    private VisionSubsystem vision = VisionSubsystem.getInstance();

    private CommandXboxController controller;
    private BezierCurve bezierCurve;
    private double maxSpeed;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
                                                                             .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private Pose2d currentPose;
    private Pose2d target;
    private PIDController rotationPID = new PIDController(5, 0, 0);
    
    
    public RotationAlign(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, BezierCurve bezierCurve, double maxSpeed) {
        drive = drivetrain;
        this.controller = controller;
        this.bezierCurve = bezierCurve;
        this.maxSpeed = maxSpeed;
        
        currentPose = drive.getState().Pose;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        target = vision.getTargetPose();
        currentPose = drive.getState().Pose;
        double rotationTarget = calculateRotationAngleTarget();

        rotationPID.setSetpoint(rotationTarget);
    }

    @Override
    public void execute() {
        currentPose = drive.getState().Pose;

        rotationPID.setSetpoint(calculateRotationAngleTarget());
        double power = rotationPID.calculate(currentPose.getRotation().getRadians());
        
        SwerveRequest request = driveRequest
        .withVelocityX(-bezierCurve.getOutput(controller.getLeftY())  * maxSpeed)
        .withVelocityY(-bezierCurve.getOutput(controller.getLeftX()) * maxSpeed)
        .withRotationalRate(power);

        drive.setControl(request);


    }

    public double calculateRotationAngleTarget() {
        double xDiff = target.getX() - currentPose.getX();
        double yDiff = target.getY() - currentPose.getY();
        double rotationTarget = Math.atan(yDiff / xDiff);

        if(currentPose.getX() > target.getX()) {
            rotationTarget += Math.PI;
        }

        return rotationTarget;

    }

    public void end(boolean interrupted) {
        SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
        drive.setControl(stop);
    }
}
