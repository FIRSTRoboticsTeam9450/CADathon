// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem INSTANCE;

  public enum ShooterState {
    SHOOTING,
    NOT_SHOOTING
  }
  ShooterState currentState = ShooterState.NOT_SHOOTING;
  
  private TalonFX motorWheelFront = new TalonFX(ShooterConstants.FRONT_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorWheelBack = new TalonFX(ShooterConstants.BACK_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorAngle = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID, RobotConstants.CANIVORE_BUS);

  private PIDController velocityPID = new PIDController(5, 0, 0);
  private double velocitySetpoint = 0;
  private boolean wheelsSpunUp = false;
  private double shooterPower = 0;

  // Motion Magic parameters
  private double velocity = 18;
  private double acceleration = 11;
  private double jerk = 400;

  // Feedforward and PIDF constants
  private double currentLimit = 110;
  private double kS = 0;
  private double kV = 0.33;
  private double kA = 0.05;
  private double kP = 90;
  private double kI = 0.001;
  private double kD = 0.35;
  private double kG = 0.001;
  private DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);

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

    motorWheelFront.getConfigurator().apply(motorConfig);
    motorWheelBack.getConfigurator().apply(motorConfig);
  }

  private void configureAngleMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    motorAngle.getConfigurator().apply(motorConfig);
  }


  private boolean zeroEncoder() {
    motorAngle.setVoltage(-1);
    if (Math.abs(motorAngle.getVelocity().getValueAsDouble()) < 0.1) {
      motorAngle.setPosition(0);
      motorAngle.setVoltage(0);
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    if (!isZeroingDone) {
      isZeroingDone = zeroEncoder();
    }
    if(currentState == ShooterState.SHOOTING) {
      shooterPower = velocityPID.calculate(motorWheelFront.getVelocity().getValueAsDouble());
      motorWheelBack.setVoltage(shooterPower);
      motorWheelFront.setVoltage(shooterPower);
      if(Math.abs(motorWheelFront.getVelocity().getValueAsDouble() - velocitySetpoint) < .5) {
        wheelsSpunUp = true;
      }
    }
    if()
  }

  public boolean shooterReady () {
    if(wheelsSpunUp && angleAtSetpoint()) {
      return true;
    }
    return false;
  }
  public boolean angleAtSetpoint() {
    return false;
  }
  public void setVelocitySetpoint(double setpoint) {
    velocityPID.setSetpoint(setpoint);
    velocitySetpoint = setpoint;
  }
  public void setShooterPower(double shooterPower) {
    this.shooterPower = shooterPower;
  }
  public static ShooterSubsystem getInsatnce() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  } 

}
