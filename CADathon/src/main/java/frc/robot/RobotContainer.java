package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  @SuppressWarnings("unused")
  private final VisionSubsystem vision;
  private final CoordinationSubsystem coordSub;
  private final ShooterSubsystem shooterSub;
  private final IntakeSubsystem intakeSub;
  private final TransferSubsystem transferSub;
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
    System.out.println(drivetrain + " I PUT DRIVE TRAIN IN");
    vision = VisionSubsystem.getInstance(drivetrain);
    coordSub = CoordinationSubsystem.getInstance();
    shooterSub = ShooterSubsystem.getInstance();
    intakeSub = IntakeSubsystem.getInstance();
    transferSub = TransferSubsystem.getInstance();
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

    DRIVER.leftTrigger()
          .onTrue(
            coordSub.setStateCommand(AbsoluteStates.INTAKING_SPEECH)
          ).onFalse(
            coordSub.setStateCommand(AbsoluteStates.STORING)
          );
    DRIVER.rightTrigger()
          .onTrue(
            coordSub.setStateCommand(AbsoluteStates.SHOOTING_SPEECH)
          ).onFalse(
            coordSub.setStateCommand(AbsoluteStates.STORING)
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

    DRIVER.y().onTrue(
      new InstantCommand(() -> intakeSub.setTargetPos(intakePos.SPEECH_BUBBLES_INTAKE, intakeStates.INTAKING))
      .andThen(new InstantCommand(() -> transferSub.setWantedState(transferStates.INTAKING)))
      ).onFalse(
        new InstantCommand(() -> intakeSub.setTargetPos(intakePos.STORE, intakeStates.NOT_RUNNING))
        .andThen(new InstantCommand(() -> transferSub.setWantedState(transferStates.STORING)))
      );

    DRIVER.b().onTrue(
      new InstantCommand(() -> shooterSub.setShooterVoltage(3))
      .alongWith(new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.OVERRIDE)))
      .alongWith(new InstantCommand(() -> intakeSub.setTargetPos(intakePos.SPEECH_BUBBLES_INTAKE, intakeStates.INTAKING))
      .alongWith(new InstantCommand(() -> transferSub.setWantedState(transferStates.FEEDING))))
      ).onFalse(
        new InstantCommand(() -> shooterSub.setShooterVoltage(0))
        .alongWith(new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.IDLING)))
        .alongWith(new InstantCommand(() -> intakeSub.setTargetPos(intakePos.STORE, intakeStates.NOT_RUNNING))
        .alongWith(new InstantCommand(() -> transferSub.setWantedState(transferStates.STORING))))
      );

    DRIVER.a().onTrue(
      new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.OVERRIDE))
      .alongWith(new InstantCommand(() -> shooterSub.setShooterVoltage(6)))
      .alongWith(new InstantCommand(() -> transferSub.setWantedState(transferStates.FEEDING)))
    ).onFalse(
      new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.IDLING))
      .alongWith(new InstantCommand(() -> shooterSub.setShooterVoltage(0)))
      .alongWith(new InstantCommand(() -> transferSub.setWantedState(transferStates.STORING)))
    );

    DRIVER.x().onTrue(
      new InstantCommand(() -> shooterSub.setVelocitySetpoint(6))
      .alongWith(new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.SHOOTING)))
    );
    DRIVER.x().onFalse(new InstantCommand(()-> shooterSub.setWantedState(AngleState.IDLING, ShooterState.IDLING)));

    DRIVER.povUp()
          .whileTrue(
           new InstantCommand(() -> shooterSub.setWantedState(AngleState.AIMING, ShooterState.IDLING))
            .alongWith(new InstantCommand(() -> shooterSub.setShooterAngleSetpoint(3)))
          );

    DRIVER.povDown()
          .whileTrue(
            new InstantCommand(() -> shooterSub.setWantedState(AngleState.AIMING, ShooterState.IDLING))
            .alongWith(new InstantCommand(() -> shooterSub.setShooterAngleSetpoint(0)))
          );

    DRIVER.povDown()
          .or(DRIVER.povUp())
          .onFalse(
            new InstantCommand(() -> shooterSub.setWantedState(AngleState.IDLING, ShooterState.IDLING))
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


  public CommandXboxController getController() {
    return DRIVER;
  }
}
