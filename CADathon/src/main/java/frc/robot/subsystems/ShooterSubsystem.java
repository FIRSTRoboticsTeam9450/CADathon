// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
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
    STORING,
    IDLING,
    AIMING
  }

  private ShooterState currentShooterState = ShooterState.IDLING;
  private AngleState currentAngleState = AngleState.STORING;
  
  private TalonFX motorWheelFront = new TalonFX(ShooterConstants.FRONT_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorWheelBack = new TalonFX(ShooterConstants.BACK_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorAngle = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID, RobotConstants.CANIVORE_BUS);

  private PIDController velocityPID = new PIDController(5, 0, 0);
  private double velocitySetpoint = 0;
  private boolean wheelsSpunUp = false;
  private double shooterPower = 0;

  private double angleSetpoint = 0;

  // Motion Magic parameters
  private double mmVelocity = 18;
  private double mmAcceleration = 11;
  private double mmJerk = 400;

  // Feedforward and PIDF constants
  private double kS = 0;
  private double kV = 0.33;
  private double kA = 0.05;
  private double kP = 90;
  private double kI = 0.001;
  private double kD = 0.35;
  private double kG = 0.001;
  private DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, mmVelocity, mmAcceleration, mmJerk);

  private boolean isZeroingDone = false;

  public ShooterSubsystem() {

    configureShooterMotors();
    configureAngleMotor();
    
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

    motorWheelFront.getConfigurator().apply(motorConfig);
    motorWheelBack.getConfigurator().apply(motorConfig);

    motorWheelBack.setControl(new StrictFollower(motorWheelFront.getDeviceID()));
  }

  private void configureAngleMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    Slot0Configs slot0Config = new Slot0Configs().withKS(kS)
                                                 .withKV(kV)
                                                 .withKA(kA)
                                                 .withKP(kP)
                                                 .withKI(kI)
                                                 .withKD(kD)
                                                 .withKG(kG);

    motorConfig.Slot0 = slot0Config;

    motorConfig.MotionMagic.MotionMagicAcceleration = mmAcceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = mmVelocity;
    motorConfig.MotionMagic.MotionMagicJerk = mmJerk;

    motorAngle.getConfigurator().apply(motorConfig);
  }


  private boolean zeroEncoder() {
    motorAngle.setVoltage(-1);
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
        shooterPower = velocityPID.calculate(motorWheelFront.getVelocity().getValueAsDouble());
        motorWheelFront.setVoltage(shooterPower);

        wheelsSpunUp = rpmWithinTolerance();
        break;

      case IDLING:
        motorWheelFront.setVoltage(0);
        break;
    }

    switch (currentAngleState) {
      case STORING:
        setShooterAngleSetpoint(0);
        motorAngle.setControl(request.withPosition(0));
        break;

      case IDLING:
        motorAngle.setControl(request.withPosition(motorAngle.getPosition().getValueAsDouble()));
        break;

      case AIMING:
        motorAngle.setControl(request.withPosition(angleSetpoint));
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
    velocityPID.setSetpoint(setpoint);
    velocitySetpoint = setpoint;
  }
  public void setShooterPower(double shooterPower) {
    this.shooterPower = shooterPower;
  }

  public void setShooterAngleSetpoint(double setpoint) {
    angleSetpoint = setpoint;
  }

  public void setWantedState(AngleState angleState, ShooterState shooterState) {
    currentAngleState = angleState;
    currentShooterState = shooterState;
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
