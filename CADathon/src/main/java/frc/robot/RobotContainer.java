package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ResetIMU;
import frc.robot.commands.RotationAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoordinationSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CoordinationSubsystem.AbsoluteStates;
import frc.robot.subsystems.CoordinationSubsystem.ScoringLocation;
import frc.robot.subsystems.IntakeSubsystem.intakePos;
import frc.robot.subsystems.IntakeSubsystem.intakeStates;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.transferStates;
import frc.robot.util.BezierCurve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final static VisionSubsystem vision = VisionSubsystem.getInstance(drivetrain);
  private final CoordinationSubsystem coordSub;
  private final ShooterSubsystem shooterSub;
  private final IntakeSubsystem intakeSub;
  private final TransferSubsystem transferSub;
  public static double pigeonOffset = 0;

  public BezierCurve driveBezier = new BezierCurve("drive", 117.4, 0.054, 91.4, 0.76, 0.1, 0.01);
  public BezierCurve rotateBezier = new BezierCurve("rotate", 120.1, 0.145, 92.2, 0.362, 0.035, 0.03);

  //These 2 should not change unless if something in Tuner constants change, EX: Different swerve modules / gear ratios
  private static final double DEFAULT_MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private static final double DEFAULT_MAX_ANGULAR_RATE = RotationsPerSecond.of(1.125).in(RadiansPerSecond); // Unsure of how this value was aquired, but was copied from Reefscape, recover2 branch

  // If you need to modify max speeds, this is most likely the one you want
  public static double maxSpeed = DEFAULT_MAX_SPEED;
  public static double maxAngularRate = DEFAULT_MAX_ANGULAR_RATE;

  private final CommandXboxController DRIVER = new CommandXboxController(0);

   private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(driveBezier.getDeadband())
        .withRotationalDeadband(rotateBezier.getDeadband())    
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SendableChooser<Command> autoChooser;
  /**
   *  The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    coordSub = CoordinationSubsystem.getInstance();
    shooterSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    transferSub = TransferSubsystem.getInstance();

    autoChooser = new SendableChooser<>();

    configureBindings();
    registerCommands();
    configureAutos();
  }

  private void configureAutos() {
    autoChooser.setDefaultOption("Drive and Shoot", drivetrain.getAutoPath("DriveUpAndShoot", false));
    autoChooser.addOption("Preload + 1 Intake", drivetrain.getAutoPath("Preload + 1 Intake", false));
    autoChooser.addOption("Run and Gun", drivetrain.getAutoPath("Run and Gun", false));
    autoChooser.addOption("Intake Line", drivetrain.getAutoPath("Intake Line", false));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void registerCommands() {
    NamedCommands.registerCommand("Store", coordSub.setStateCommand(AbsoluteStates.STORING));
    NamedCommands.registerCommand("Shoot", coordSub.setStateCommand(AbsoluteStates.PREPARING_FOR_SHOT));
    NamedCommands.registerCommand("Intake", coordSub.setStateCommand(AbsoluteStates.INTAKING_SPEECH));

    NamedCommands.registerCommand("IntakeUp", coordSub.setDoesIntakeRaiseCommand(true));
    NamedCommands.registerCommand("IntakeDown", coordSub.setDoesIntakeRaiseCommand(false));
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
    DRIVER.leftStick().whileTrue(
      new AutoAlign(drivetrain, 
                    DRIVER, 
                    new Pose2d(new Translation2d(3.52, 5.84), new Rotation2d((-60.0/180.0) * Math.PI))
                    )
    );


    DRIVER.leftTrigger().onTrue(
      coordSub.leftTriggerCommand(drivetrain, DRIVER, driveBezier, maxSpeed)
    ).onFalse(
      coordSub.setStateCommand(AbsoluteStates.STORING)
      .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()))
    );
    DRIVER.rightTrigger().whileTrue(
      coordSub.rightTriggerHeld(true)
    ).onFalse(
      coordSub.rightTriggerHeld(false)
    );
    DRIVER.leftBumper()
          .onTrue(
            coordSub.setStateCommand(AbsoluteStates.INTAKING_STORY)
          ).onFalse(
            coordSub.setStateCommand(AbsoluteStates.STORING)
          );
    DRIVER.rightBumper()
          .onTrue(
            coordSub.setStateCommand(AbsoluteStates.SHOOT_STORY)
          ).onFalse(
            coordSub.setStateCommand(AbsoluteStates.STORING)
          );

    // DRIVER.y()
    //       .onTrue(
    //         coordSub.setScoringLocationCommand(ScoringLocation.FOOTHILLS_HIGH)
    //       );
    // DRIVER.b()
    //       .onTrue(
    //         coordSub.setScoringLocationCommand(ScoringLocation.UPTOWN)
    //       );
    // DRIVER.x()
    //       .onTrue(
    //         coordSub.setScoringLocationCommand(ScoringLocation.DOWNTOWN)
    //       );
    // DRIVER.a()
    //       .onTrue(
    //         coordSub.setScoringLocationCommand(ScoringLocation.FOOTHILLS_LOW)
    //       );

    DRIVER.povLeft().onTrue(
      coordSub.setDoesIntakeRaiseCommand(true)
    );

    DRIVER.start().onTrue(
      new ResetIMU(drivetrain)
    );

    DRIVER.povRight().onTrue(
      coordSub.setStateCommand(AbsoluteStates.REJECTING)
    ).onFalse(
      coordSub.setStateCommand(AbsoluteStates.STORING)
    );

    DRIVER.b().onTrue(
      coordSub.setStateCommand(AbsoluteStates.SHOOTER_OVERRIDE)
    ).onFalse(
      coordSub.setStateCommand(AbsoluteStates.STORING)
    );


    //look for .until() for having shooter wait to feed till shooter is at wanted Veloc and angle
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  public CommandXboxController getController() {
    return DRIVER;
  }
}
