package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.TransferConstants;

/**
 * Subsystem for The Transfer System of the robot, This includes managing the storage and movement of game peices
 * <p>
 *  The Transfer Subsystem handles both the Hopper and Tower
 * </p>
 */
public class TransferSubsystem extends SubsystemBase {

  //Instance of Subsystem
  private static TransferSubsystem INSTANCE;

  public enum transferStates {
    STORING,
    INTAKING,
    PREPARING_FOR_SHOT,
    FEEDING,
    REJECTING,
  }

  private transferStates currentState = transferStates.STORING;
  

  /* ---------- Devices ---------- */
  private TalonFX motorTowerWheels = new TalonFX(TransferConstants.TOWER_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorRightSideRollers = new TalonFX(TransferConstants.RIGHT_SIDE_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorLeftSideRollers = new TalonFX(TransferConstants.LEFT_SIDE_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorBottomRollers = new TalonFX(TransferConstants.BOTTOM_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private CANrange canrange = new CANrange(TransferConstants.TOWER_CANRANGE_ID, RobotConstants.CANIVORE_BUS);

  private boolean detectedSpeech;
  private Timer timer = new Timer();
  private boolean runForward = false;

  public TransferSubsystem() {
    configureTower();
    configureHopper();
  }

  private void configureTower() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();
    canrangeConfig.ProximityParams.ProximityThreshold = 0.07; // meters, 0.15 is roughly half way, which is when we want to see the ball
    // canrangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement //This should be used, but will need to see what the value should be when robot is built

    motorTowerWheels.getConfigurator().apply(motorConfig);
    canrange.getConfigurator().apply(canrangeConfig);
  }

  private void configureHopper() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    motorRightSideRollers.getConfigurator().apply(motorConfig);
    motorBottomRollers.getConfigurator().apply(motorConfig);

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorLeftSideRollers.getConfigurator().apply(motorConfig);
  }
  
  @Override
  public void periodic() {

    Logger.recordOutput("HeroHeist/Transfer/CurrentState", currentState);
    //detectedSpeech = ;
    if(!getCANRangeTriggered() && currentState == transferStates.STORING) {
      runForward = false;
    }
    applyStates();
    publishLogs();

    Logger.recordOutput("HeroHeist/Transfer/Tower/Ball Detected?", getCANRangeTriggered());
    
  }

  private void applyStates() {
    
    double hopperBottomVoltage = 0;
    double hopperSideVoltage = 0;
    double towerVoltage = 0;

    switch (currentState) {
      case STORING:
        hopperBottomVoltage = 0;
        hopperSideVoltage = 0;
        towerVoltage = 0;
        break;

      case INTAKING:
        hopperBottomVoltage = 3;
        hopperSideVoltage = 3;
        if (getCANRangeTriggered() && !runForward) {
          towerVoltage = 0.75;
          timer.restart();
          runForward = true;
        } else if (runFowardDone()) {
          hopperBottomVoltage = 1;
          towerVoltage = 0;
          hopperSideVoltage = 0;
          timer.stop();
        } else if(!runForward){
          towerVoltage = 1;
        }
        break;
      
      case PREPARING_FOR_SHOT:
        hopperBottomVoltage = 4;
        hopperSideVoltage = 4;
        if (getCANRangeTriggered() && !runForward) {
          towerVoltage = 0.75;
          timer.restart();
          runForward = true;
        } else if (runFowardDone()) {
          towerVoltage = 0;
          hopperSideVoltage = 0;
          timer.stop();
        } else if(!runForward){
          towerVoltage = 1;
        }
        break;

      case FEEDING:
        hopperBottomVoltage = 6;
        hopperSideVoltage = -6;
        towerVoltage = 6;
        break;

      case REJECTING:
        hopperBottomVoltage = -3;
        hopperSideVoltage = -3;
        towerVoltage = -1;
        break;

      default:
        hopperBottomVoltage = 0;
        hopperSideVoltage = 0;
        towerVoltage = 0;
        break;
    }
    
    Logger.recordOutput("HeroHeist/Transfer/Debugging/Hopper Voltage", hopperBottomVoltage);
    Logger.recordOutput("HeroHeist/Transfer/Debugging/Tower Voltage", towerVoltage);

    motorBottomRollers.setVoltage(hopperBottomVoltage);
    motorRightSideRollers.setVoltage(hopperSideVoltage);
    motorLeftSideRollers.setVoltage(hopperSideVoltage);
    motorTowerWheels.setVoltage(towerVoltage);
  }

  public boolean runFowardDone() {
    if(timer.get() > 2) {
      return true;
    }
    return false;
  }

  private void publishLogs() {
    Logger.recordOutput("HeroHeist/Transfer/Tower/Ball Detected?", detectedSpeech);
    Logger.recordOutput("HeroHeist/Transfer/CurrentState", currentState);
  }

  public void setWantedState(transferStates wantedState) {
    currentState = wantedState;
  }

  public transferStates getCurrentState() {
    return currentState;
  }

  public boolean getCANRangeTriggered() {
    return canrange.getIsDetected(true).getValue().booleanValue();
  }

  public static TransferSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new TransferSubsystem();
    }
    return INSTANCE;
  }

}
