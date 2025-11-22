package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

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

  private HashMap<transferStates, double[]> stateVoltageMap = new HashMap<>();
  

  /* ---------- Devices ---------- */
  private TalonFX motorTowerWheels = new TalonFX(TransferConstants.TOWER_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorSideRollers = new TalonFX(TransferConstants.SIDE_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorBottomRollers = new TalonFX(TransferConstants.BOTTOM_ROLLERS_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private CANrange canrange = new CANrange(TransferConstants.TOWER_CANRANGE_ID, RobotConstants.CANIVORE_BUS);

  private boolean hasStateChanged;

  public TransferSubsystem() {

    configureTower();
    configureHopper();

    configureMap();

    hasStateChanged = false;

  }

  private void configureMap() {

    double[] tmpArr = {0.0, 0.0};
    stateVoltageMap.put(transferStates.STORING, tmpArr);

    tmpArr[0] = 4.0;
    tmpArr[1] = -2;
    stateVoltageMap.put(transferStates.INTAKING, tmpArr);

    tmpArr[0] = 4.0;
    tmpArr[1] = 2.0;
    stateVoltageMap.put(transferStates.PREPARING_FOR_SHOT, tmpArr);

    tmpArr[0] = 8;
    tmpArr[1] = 8;
    stateVoltageMap.put(transferStates.FEEDING, tmpArr);

    tmpArr[0] = -12;
    tmpArr[1] = -12;
    stateVoltageMap.put(transferStates.REJECTING, tmpArr);

  }

  private void configureTower() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50;

    CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();
    canrangeConfig.ProximityParams.ProximityThreshold = 0.15; // meters, 0.15 is roughly half way, which is when we want to see the ball
    // canrangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement //This should be used, but will need to see what the value should be when robot is built

    motorTowerWheels.getConfigurator().apply(motorConfig);
    canrange.getConfigurator().apply(canrangeConfig);
  }

  private void configureHopper() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 50;

    motorSideRollers.getConfigurator().apply(motorConfig);
    motorBottomRollers.getConfigurator().apply(motorConfig);
  }
  
  @Override
  public void periodic() {

    if (hasStateChanged) {
      applyStates();
      hasStateChanged = false;
    }
    
  }

  private void applyStates() {

    double[] voltageArr = stateVoltageMap.get(currentState);
    
    double hopperVoltage = voltageArr[0];
    double towerVoltage = voltageArr[1];

    motorBottomRollers.setVoltage(hopperVoltage);
    motorSideRollers.setVoltage(hopperVoltage);
    motorTowerWheels.setVoltage(towerVoltage);
  }

  public void setWantedState(transferStates wantedState) {
    currentState = wantedState;
    hasStateChanged = true;
  }

  public transferStates getCurrentState() {
    return currentState;
  }

  public static TransferSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new TransferSubsystem();
    }
    return INSTANCE;
  }

}
