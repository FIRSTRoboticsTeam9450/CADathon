package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.intakePos;
import frc.robot.subsystems.IntakeSubsystem.intakeStates;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TransferSubsystem.transferStates;

public class CoordinationSubsystem extends SubsystemBase{
    
    private static CoordinationSubsystem INSTANCE;

    private IntakeSubsystem intakeInstance = IntakeSubsystem.getInstance();
    private TransferSubsystem transferInstance = TransferSubsystem.getInstance();
    private ShooterSubsystem shooterInstance = ShooterSubsystem.getInstance();
    private VisionSubsystem visionInstance = VisionSubsystem.getInstance();

    public enum AbsoluteStates {
        SHOOTER_OVERRIDE,
        STORING,
        INTAKING_SPEECH,
        INTAKING_STORY,
        PREPARING_FOR_SHOT,
        SHOOTING_SPEECH,
        SHOOT_STORY,
        REJECTING
    }

    public enum ScoringLocation {
        UPTOWN,
        DOWNTOWN,
        FOOTHILLS_LOW,
        FOOTHILLS_HIGH
    }

    private AbsoluteStates currentState = AbsoluteStates.STORING;

    private ScoringLocation wantedScoringLocation = ScoringLocation.DOWNTOWN;

    public CoordinationSubsystem() {

        applyState();
        
        publishLogs();
    }

    private void applyState() {

        transferStates transferState = transferStates.STORING;
        intakePos intakeState = intakePos.STORE;
        intakeStates intaking = intakeStates.NOT_RUNNING;
        ShooterState shooterState = ShooterState.IDLING;
        AngleState shooterAngleState = AngleState.IDLING;
        
        switch (currentState) {
            case SHOOTER_OVERRIDE:
                shooterState = ShooterState.SHOOTING;
                shooterAngleState = AngleState.OVERRIDE;
                
            case STORING:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;
            
            case INTAKING_SPEECH:
                transferState = transferStates.INTAKING;
                intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                intaking = intakeStates.INTAKING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;
            
            case INTAKING_STORY:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORY_BOARDS_INTAKE;
                intaking = intakeStates.INTAKING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;
            
            case PREPARING_FOR_SHOT:
                    transferState = transferStates.PREPARING_FOR_SHOT;
                    intakeState = intakePos.STORE;
                    intaking = intakeStates.NOT_RUNNING;
                    shooterState = ShooterState.SHOOTING;
                    shooterAngleState = AngleState.AIMING;
                    transferState = transferStates.STORING;
                    intakeState = intakePos.STORE;
                    intaking = intakeStates.NOT_RUNNING;
                    shooterState = ShooterState.SHOOTING;
                    shooterAngleState = AngleState.AIMING;
                break;

            case SHOOTING_SPEECH:
                transferState = transferStates.FEEDING;
                intakeState = intakePos.STORY_BOARDS_SCORE;
                intaking = intakeStates.OUTTAKING;
                shooterState = ShooterState.SHOOTING;
                shooterAngleState = AngleState.AIMING;
                break;

            case SHOOT_STORY:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                shooterState = ShooterState.IDLING;
                shooterAngleState = AngleState.IDLING;
                break;

            case REJECTING:
                transferState = transferStates.REJECTING;
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
        Logger.recordOutput("HeroHesit/Coordination/Faults/Intake Instance null?", intakeInstance == null);
        Logger.recordOutput("HeroHeist/Coordination/Faults/Transfer Instance null?", transferInstance == null);
        Logger.recordOutput("HeroHeist/Coordination/Faults/Shooter Instance null?", shooterInstance == null);
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
