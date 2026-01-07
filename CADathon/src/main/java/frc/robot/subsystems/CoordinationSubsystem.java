package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.RotationAlign;
import frc.robot.subsystems.IntakeSubsystem.intakePos;
import frc.robot.subsystems.IntakeSubsystem.intakeStates;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.transferStates;
import frc.robot.util.BezierCurve;

public class CoordinationSubsystem extends SubsystemBase{
    
    private static CoordinationSubsystem INSTANCE;

    private IntakeSubsystem intakeInstance = IntakeSubsystem.getInstance();
    private TransferSubsystem transferInstance = TransferSubsystem.getInstance();
    private ShooterSubsystem shooterInstance = ShooterSubsystem.getInstance();
    private VisionSubsystem visionInstance = RobotContainer.vision;

    public enum AbsoluteStates {
        SHOOTER_OVERRIDE,
        STORING,
        INTAKING_SPEECH,
        INTAKING_STORY,
        PREPARING_FOR_SHOT,
        SHOOTING_SPEECH,
        SHOOT_STORY,
        REJECTING,
        PREPARING_FOR_SHOT_OVERRIDE
    }

    public enum ScoringLocation {
        UPTOWN,
        DOWNTOWN,
        FOOTHILLS_LOW,
        FOOTHILLS_HIGH
    }

    private AbsoluteStates currentState = AbsoluteStates.STORING;

    private ScoringLocation wantedScoringLocation = ScoringLocation.DOWNTOWN;

    private boolean intakeRaise = true;
    private boolean triggerModeIsShooting = false;

    public CoordinationSubsystem() {
    }

    @Override
    public void periodic() {
        applyState();
        
        publishLogs();
    }

    private void applyState() {

        transferStates transferState = transferStates.STORING;
        intakePos intakeState = intakePos.STORE;
        intakeStates intaking = intakeStates.NOT_RUNNING;
        ShooterState shooterState = ShooterState.IDLING;
        AngleState shooterAngleState = AngleState.IDLING;
        
        if(currentState == AbsoluteStates.PREPARING_FOR_SHOT) {
            if(shooterInstance.shooterReady()) {
                currentState = AbsoluteStates.SHOOTING_SPEECH;
            }
        } else if (currentState == AbsoluteStates.SHOOTING_SPEECH) {
            if (!shooterInstance.shooterReady()) {
                currentState = AbsoluteStates.PREPARING_FOR_SHOT;
            }
        }
        if(currentState == AbsoluteStates.PREPARING_FOR_SHOT_OVERRIDE) {
            if(shooterInstance.shooterReady()) {
                currentState = AbsoluteStates.SHOOTER_OVERRIDE;
            }
        } else if (currentState == AbsoluteStates.SHOOTER_OVERRIDE) {
            if (!shooterInstance.shooterReady()) {
                currentState = AbsoluteStates.PREPARING_FOR_SHOT_OVERRIDE;
            }
        }
        switch (currentState) {
            case PREPARING_FOR_SHOT_OVERRIDE:
                shooterState = ShooterState.OVERRIDE;
                shooterAngleState = AngleState.OVERRIDE;

                transferState = transferStates.PREPARING_FOR_SHOT;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.NOT_RUNNING;
                break;
            
            case SHOOTER_OVERRIDE:
                shooterState = ShooterState.OVERRIDE;
                shooterAngleState = AngleState.OVERRIDE;

                transferState = transferStates.FEEDING;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.INTAKE_SLOW;
                break;

            case STORING:
                transferState = transferStates.STORING;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.AIMING;
                break;
            
            case INTAKING_SPEECH:
                intakeRaise = false;
                transferState = transferStates.INTAKING;
                intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                intaking = intakeStates.INTAKING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.AIMING;
                break;
            
            case INTAKING_STORY:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORY_BOARDS_INTAKE;
                intaking = intakeStates.INTAKING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.AIMING;
                break;
            
            case PREPARING_FOR_SHOT:
                transferState = transferStates.PREPARING_FOR_SHOT;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.SHOOTING;
                shooterAngleState = AngleState.AIMING;
                break;

            case SHOOTING_SPEECH:
                transferState = transferStates.FEEDING;
                intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                intaking = intakeStates.INTAKING;
                shooterState = ShooterState.SHOOTING;
                shooterAngleState = AngleState.AIMING; // Used to be aiming
                break;

            case SHOOT_STORY:
                transferState = transferStates.STORING;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;

            case REJECTING:
                transferState = transferStates.REJECTING;
                if (intakeRaise) {
                    intakeState = intakePos.STORE;
                } else {
                    intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                }
                intaking = intakeStates.OUTTAKING;
                shooterAngleState = AngleState.AIMING;
                break;
            
            default:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;
        }

        intakeInstance.setTargetPos(intakeState, intaking);
        transferInstance.setWantedState(transferState);
        shooterInstance.setWantedState(shooterAngleState, shooterState);
        
    }

    private void publishLogs() {
        Logger.recordOutput("HeroHeist/Coordination/Faults/Intake Instance null?", intakeInstance == null);
        Logger.recordOutput("HeroHeist/Coordination/Faults/Transfer Instance null?", transferInstance == null);
        Logger.recordOutput("HeroHeist/Coordination/Faults/Shooter Instance null?", shooterInstance == null);
        Logger.recordOutput("HeroHeist/Coordination/Current State", currentState);
        Logger.recordOutput("HeroHeist/Coordination/Wanted Scoring Type", wantedScoringLocation);
        Logger.recordOutput("HeroHeist/Coordination/Should Intake Raise", intakeRaise);
    }

    public Command rightTriggerHeld(boolean held) {
        return new InstantCommand(() -> triggerModeIsShooting = held);
    }

    public Command leftTriggerCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController DRIVER, BezierCurve driveBezier, double maxSpeed) {
        return Commands.either(
            setStateCommand(AbsoluteStates.PREPARING_FOR_SHOT)
                .andThen(new RotationAlign(drivetrain, DRIVER, driveBezier, maxSpeed)), 
            setStateCommand(AbsoluteStates.INTAKING_SPEECH),
            () -> triggerModeIsShooting);
    }

    public void setState(AbsoluteStates wantedState) {
        currentState = wantedState;
    }

    public Command setStateCommand(AbsoluteStates wantedState) {
        return new InstantCommand(() -> setState(wantedState));
    }

    public void setScoringLocation(ScoringLocation location) {
        wantedScoringLocation = location;
    }

    public Command setScoringLocationCommand(ScoringLocation location) {
        return new InstantCommand(() -> setScoringLocation(location));
    }

    private void setDoesIntakeRaise(boolean value) {
        intakeRaise = value;
    }

    public Command setDoesIntakeRaiseCommand(boolean value) {
        return new InstantCommand(() -> setDoesIntakeRaise(value));
    }
    
    public AbsoluteStates getCurrentState() {
        return currentState;
    }

    public ScoringLocation getScoringLocation() {
        return wantedScoringLocation;
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
        }
        return INSTANCE;
    }

}
