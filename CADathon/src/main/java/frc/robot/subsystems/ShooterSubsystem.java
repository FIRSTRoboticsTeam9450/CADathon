// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem INSTANCE;

  public enum ShooterState {
    IDLING,
    SHOOTING
  }

  public enum AngleState {
    OVERRIDE,
    STORING,
    IDLING,
    AIMING
  }

  private ShooterState currentShooterState = ShooterState.IDLING;
  private AngleState currentAngleState = AngleState.STORING;
  
  private TalonFX motorWheelFront = new TalonFX(ShooterConstants.FRONT_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorWheelBack = new TalonFX(ShooterConstants.BACK_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorAngle = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID, RobotConstants.CANIVORE_BUS);

  private boolean wheelsSpunUp = false;

  private double angleSetpoint = 0;
  private double angleVoltage = 0;

  // Motion Magic parameters
  private final double mmVelocity = 2;
  private final double mmAcceleration = 1;
  private final double mmJerk = 400;

  // Feedforward and PIDF constants
  private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kS", 0);
  private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kV", 0.33);
  private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kA", 0.05);
  private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kP", 0.01);
  private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kI", 0);
  private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kD", 0);
  private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Shooter/Angle/kG", 0.05);
  private double mmKS = logMMKS.get();
  private double mmKV = logMMKV.get();
  private double mmKA = logMMKA.get();
  private double mmKP = logMMKP.get();
  private double mmKI = logMMKI.get();
  private double mmKD = logMMKD.get();
  private double mmKG = logMMKG.get();
  private DynamicMotionMagicVoltage mmRequest;

  private LoggedNetworkNumber logVKS = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kS", 0.1);
  private LoggedNetworkNumber logVKV = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kV", 0.12);
  private LoggedNetworkNumber logVKP = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kP", 0.11);
  private LoggedNetworkNumber logVKFF = new LoggedNetworkNumber("/Tuning/Shooter/Outtake/kFF", 0.5);
  private double vKS = logVKS.get();
  private double vKV = logVKV.get();
  private double vKP = logVKP.get();
  private double vKFF = logVKFF.get();
  private final int vSlot = 0;
  private final VelocityVoltage vRequest;
  private double velocitySetpoint = 0;


  private final double maxdHoodVoltage;

  private boolean isZeroingDone = false;

  public ShooterSubsystem() {

    configureShooterMotors();
    configureAngleMotor();

    mmRequest = new DynamicMotionMagicVoltage(0, mmVelocity, mmAcceleration, mmJerk);
    vRequest = new VelocityVoltage(0).withSlot(vSlot);

    maxdHoodVoltage = 0.15;
    
    isZeroingDone = false;

  }

  private void configureShooterMotors() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 3;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    motorConfig.Feedback.SensorToMechanismRatio = 9/8; //1.125

    Slot0Configs slot0Config = new Slot0Configs().withKS(vKS)
                                                 .withKV(vKV)
                                                 .withKP(vKP);

    motorConfig.Slot0 = slot0Config;

    motorWheelFront.getConfigurator().apply(motorConfig);
    motorWheelBack.getConfigurator().apply(motorConfig);

    motorWheelBack.setControl(new StrictFollower(motorWheelFront.getDeviceID()));
  }

  private void configureAngleMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 20;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.3;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 10;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 5;

    Slot0Configs slot0Config = new Slot0Configs().withKS(mmKS)
                                                 .withKV(mmKV)
                                                 .withKA(mmKA)
                                                 .withKP(mmKP)
                                                 .withKI(mmKI)
                                                 .withKD(mmKD)
                                                 .withKG(mmKG);

    motorConfig.Slot0 = slot0Config;

    motorConfig.MotionMagic.MotionMagicAcceleration = mmAcceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = mmVelocity;
    motorConfig.MotionMagic.MotionMagicJerk = mmJerk;

    // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    // motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2; //Set this to however many rotations the motor encoder reads when the hood is just about to run off the gears.

    motorAngle.getConfigurator().apply(motorConfig);
  }

  private void updateShooterAngleConstants() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    Slot0Configs slot0Config = new Slot0Configs().withKS(mmKS)
                                                 .withKV(mmKV)
                                                 .withKA(mmKA)
                                                 .withKP(mmKP)
                                                 .withKI(mmKI)
                                                 .withKD(mmKD)
                                                 .withKG(mmKG);
    motorConfig.Slot0 = slot0Config;

    motorAngle.getConfigurator().apply(motorConfig);
  }

  private void updateShooterVelocityConstants() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    Slot0Configs slot0Config = new Slot0Configs().withKS(vKS)
                                                 .withKV(vKV)
                                                 .withKP(vKP);

    motorConfig.Slot0 = slot0Config;

    motorWheelFront.getConfigurator().apply(motorConfig);
    motorWheelBack.getConfigurator().apply(motorConfig);
  }


  private boolean zeroEncoder() {
    motorAngle.setVoltage(-0.04);
    if (Math.abs(motorAngle.getVelocity().getValueAsDouble()) < 0.1) {
      motorAngle.setPosition(0);
      motorAngle.setVoltage(0);
      currentAngleState = AngleState.IDLING;
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (!isZeroingDone) {
      currentAngleState = AngleState.STORING;
      isZeroingDone = zeroEncoder();
    }
    applyState();
  }

  private void applyState() {
    switch (currentShooterState) {
      case SHOOTING:
        motorWheelFront.setControl(vRequest.withVelocity(velocitySetpoint).withFeedForward(vKFF));

        wheelsSpunUp = rpmWithinTolerance();
        break;

      case IDLING:
        motorWheelFront.setVoltage(0);
        break;
    }

    switch (currentAngleState) {
      case OVERRIDE:
        motorAngle.setVoltage(angleVoltage);
        break;

      case STORING:
        setShooterAngleSetpoint(0);
        motorAngle.setControl(mmRequest.withPosition(0));
        break;

      case IDLING:
        motorAngle.setControl(mmRequest.withPosition(motorAngle.getPosition().getValueAsDouble()));
        break;

      case AIMING:
        motorAngle.setControl(mmRequest.withPosition(angleSetpoint));
        break;
    }
  }

  /* --------------- Calculations --------------- */

  /**
   * uses a default tolerance
   * @return
   */
  private boolean rpmWithinTolerance() {
    return rpmWithinTolerance(.5);
  }

  /**
   * Calculates if the RPS of the shooter is withing a certain range of where we wish it to be
   * @param tolerance rotation per second off it can be from where we want it to be
   * @return if withing tolerance
   */
  private boolean rpmWithinTolerance(double tolerance) {
    return Math.abs(velocitySetpoint - motorWheelFront.getVelocity().getValueAsDouble()) < tolerance;
  }

  /* --------------- Setters --------------- */

  public void setVelocitySetpoint(double setpoint) {
    velocitySetpoint = setpoint;
  }

  public void setShooterAngleSetpoint(double setpoint) {
    angleSetpoint = setpoint;
  }

  public void setWantedState(AngleState angleState, ShooterState shooterState) {
    currentAngleState = angleState;
    currentShooterState = shooterState;
  }

  /**
   * Only takes effect if Angle state is in OVERRIDE, if not then it'll use PID-type controlling
   * @param voltage
   */
  public void setAngleVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -maxdHoodVoltage, maxdHoodVoltage);
    angleVoltage = voltage;
  }

  public void updateShooterConstants() {
    boolean updateOuttakeVals = false;
    boolean updateAngleVals = false;

    double lVKSVal = logVKS.get();
    double lVKVVal = logVKV.get();
    double lVKPVal = logVKP.get();
    double lVKFFVal = logVKFF.get();

    updateOuttakeVals = (vKS != lVKSVal)
                     || (vKV != lVKVVal)
                     || (vKP != lVKPVal)
                     || (vKFF != lVKFFVal);

    if (updateOuttakeVals) {
      vKS = lVKSVal;
      vKV = lVKVVal;
      vKP = lVKPVal;
      vKFF = lVKFFVal;
      updateShooterVelocityConstants();
    }

    double lMMKSVal = logMMKS.get();
    double lMMKVVal = logMMKV.get();
    double lMMKAVal = logMMKA.get();
    double lMMKPVal = logMMKP.get();
    double lMMKIVal = logMMKI.get();
    double lMMKDVal = logMMKD.get();
    double lMMKGVal = logMMKG.get();

    updateAngleVals = (mmKS != lMMKSVal)
                   || (mmKV != lMMKVVal)
                   || (mmKA != lMMKAVal)
                   || (mmKP != lMMKPVal)
                   || (mmKI != lMMKIVal)
                   || (mmKD != lMMKDVal)
                   || (mmKG != lMMKGVal);

    if (updateAngleVals) {
      mmKS = lMMKSVal;
      mmKV = lMMKVVal;
      mmKA = lMMKAVal;
      mmKP = lMMKPVal;
      mmKI = lMMKIVal;
      mmKD = lMMKDVal;
      mmKG = lMMKGVal;

      updateShooterAngleConstants();
    }
    
  }

  public void forceHoodZero() {
    if (DriverStation.isDisabled()) {
      motorAngle.setPosition(0);
    }
  }

  /* --------------- Getters -------------- */

  public boolean shooterReady () {
    if(wheelsSpunUp && angleAtSetpoint()) {
      return true;
    }
    return false;
  }

  public boolean angleAtSetpoint() {
    return false;
  }

  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  } 

}
