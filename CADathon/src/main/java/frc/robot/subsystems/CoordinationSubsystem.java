package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.intakePos;
import frc.robot.subsystems.IntakeSubsystem.intakeStates;
import frc.robot.subsystems.TransferSubsystem.transferStates;

public class CoordinationSubsystem extends SubsystemBase{
    
    private static CoordinationSubsystem INSTANCE;

    private IntakeSubsystem intakeInstance = IntakeSubsystem.getInstance();
    private TransferSubsystem transferInstance = TransferSubsystem.getInstance();

    public enum AbsoluteStates {
        STORING,
        INTAKING_SPEECH,
        INTAKING_STORY,
        PREPARING_FOR_SHOT,
        SHOOTING_SPEECH,
        SHOOT_STORY,
        REJECTING
    }

    private AbsoluteStates currentState = AbsoluteStates.STORING;
    private AbsoluteStates previousState = currentState;

    private boolean hasStateChanged = false;

    public CoordinationSubsystem() {

        if (hasStateChanged) {
            applyState();
        }

    }

    private void applyState() {

        transferStates transferState = transferStates.STORING;
        intakePos intakeState = intakePos.STORE;
        intakeStates intaking = intakeStates.NOT_RUNNING;

        switch (currentState) {
            case STORING:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                break;
            
            case INTAKING_SPEECH:
                transferState = transferStates.INTAKING;
                intakeState = intakePos.SPEECH_BUBBLES_INTAKE;
                intaking = intakeStates.INTAKING;
                break;
            
            case INTAKING_STORY:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORY_BOARDS_INTAKE;
                intaking = intakeStates.INTAKING;
                break;
            
            case PREPARING_FOR_SHOT:
                transferState = transferStates.PREPARING_FOR_SHOT;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                break;

            case SHOOTING_SPEECH:
                transferState = transferStates.FEEDING;
                intakeState = intakePos.STORY_BOARDS_INTAKE; // CHANGE
                intaking = intakeStates.OUTTAKING;
                break;

            case SHOOT_STORY:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                break;

            case REJECTING:
                transferState = transferStates.REJECTING;
                break;
            
            default:
                transferState = transferStates.STORING;
                intakeState = intakePos.STORE;
                intaking = intakeStates.NOT_RUNNING;
                break;
        }

        intakeInstance.goToPos(intakeState, intaking);
        transferInstance.setWantedState(transferState);
    }

    public void setState(AbsoluteStates wantedState) {
        previousState = currentState;
        currentState = wantedState;
        hasStateChanged = true;
    }

    public static CoordinationSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CoordinationSubsystem();
        }
        return INSTANCE;
    }

}
