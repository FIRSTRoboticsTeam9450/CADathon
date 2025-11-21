package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TransferSubsystem.States;

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

        States transferState;

        switch (currentState) {
            case STORING:
                transferState = States.STORING;
                break;
            
            case INTAKING_SPEECH:
                transferState = States.INTAKING;
                break;
            
            case INTAKING_STORY:
                transferState = States.STORING;
                break;
            
            case PREPARING_FOR_SHOT:
                transferState = States.PREPARING_FOR_SHOT;
                break;

            case SHOOTING_SPEECH:
                transferState = States.FEEDING;
                break;

            case SHOOT_STORY:
                transferState = States.STORING;
                break;

            case REJECTING:
                transferState = States.REJECTING;
                break;
            
            default:
                transferState = States.STORING;
                break;
        }

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
