package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetIMU extends Command {
    
    CommandSwerveDrivetrain drivetrain;

    public ResetIMU(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.seedFieldCentric();
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            RobotContainer.pigeonOffset = drivetrain.getPigeon2().getRotation2d().getDegrees();
        } else {
            RobotContainer.pigeonOffset = drivetrain.getPigeon2().getRotation2d().getDegrees() + 180;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
