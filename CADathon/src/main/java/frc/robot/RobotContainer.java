package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.BezierCurve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final VisionSubsystem vision;

  public static double pigeonOffset = 0;

  public BezierCurve driveBezier = new BezierCurve("drive", 117.4, 0.054, 91.4, 0.76, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 120.1, 0.145, 92.2, 0.362, 0.035, 0.03);

  //These 2 should not change unless if something in Tuner constants change, EX: Different swerve modules / gear ratios
  private static final double DEFAULT_MAX_SPEED = 5.14; //Value comes from kSpeedAt12Volts in tuner Constants
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(1.125).in(RadiansPerSecond); // Unsure of how this value was aquired, but was copied from Reefscape, recover2 branch

  // If you need to modify max speeds, this is most likely the one you want
  public static double maxSpeed = DEFAULT_MAX_SPEED;
  public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE;

  private final CommandXboxController DRIVER = new CommandXboxController(0);

   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(driveBezier.getDeadband())
        .withRotationalDeadband(rotateBezier.getDeadband())    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /**
   *  The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    vision = VisionSubsystem.getInstance(drivetrain);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveBezier.getOutput(DRIVER.getLeftY())  * maxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveBezier.getOutput(DRIVER.getLeftX()) * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotateBezier.getOutput(DRIVER.getRightX()) * maxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
