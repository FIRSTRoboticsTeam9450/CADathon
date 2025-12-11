package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
  private TalonFX motorSideRollers = new TalonFX(TransferConstants.SIDE_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorBottomRollers = new TalonFX(TransferConstants.BOTTOM_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private CANrange canrange = new CANrange(TransferConstants.TOWER_CANRANGE_ID, RobotConstants.CANIVORE_BUS);

  private boolean detectedSpeech;

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
    canrangeConfig.ProximityParams.ProximityThreshold = 0.075; // meters, 0.15 is roughly half way, which is when we want to see the ball
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

    motorSideRollers.getConfigurator().apply(motorConfig);
    motorBottomRollers.getConfigurator().apply(motorConfig);
  }
  
  @Override
  public void periodic() {

    Logger.recordOutput("HeroHeist/TransferSubsystem/CurrentState", currentState);
    detectedSpeech = canrange.getIsDetected(true).getValue().booleanValue();

      applyStates();

    Logger.recordOutput("HeroHeist/TransferSubsystem/Tower/Ball Detected?", detectedSpeech);
    
  }

  private void applyStates() {
    
    double hopperVoltage = 0;
    double towerVoltage = 0;

    switch (currentState) {
      case STORING:
        hopperVoltage = 0;
        towerVoltage = 0;
        break;

      case INTAKING:
        hopperVoltage = 3;
        if (detectedSpeech) {
          towerVoltage = 0;
        } else {
          towerVoltage = 0.75;
        }
        break;
      
      case PREPARING_FOR_SHOT:
        hopperVoltage = 4;
        if (detectedSpeech) {
          towerVoltage = 0;
        } else {
          towerVoltage = 2;
        }
        break;

      case FEEDING:
        hopperVoltage = 4;
        towerVoltage = 4;
        break;

      case REJECTING:
        hopperVoltage = -3;
        towerVoltage = -1;
        break;

      default:
        hopperVoltage = 0;
        towerVoltage = 0;
        break;
    }
    
    Logger.recordOutput("HeroHeist/TransferSubsystem/Debugging/Hopper Voltage", hopperVoltage);
    Logger.recordOutput("HeroHeist/TransferSubsystem/Debugging/Tower Voltage", towerVoltage);

    motorBottomRollers.setVoltage(hopperVoltage);
    motorSideRollers.setVoltage(hopperVoltage);
    motorTowerWheels.setVoltage(towerVoltage);
  }

  public void setWantedState(transferStates wantedState) {
    currentState = wantedState;
  }

  public transferStates getCurrentState() {
    return currentState;
  }

  public boolean getCANRangeTriggered() {
    return detectedSpeech;
  }

  public static TransferSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new TransferSubsystem();
    }
    return INSTANCE;
  }

}
