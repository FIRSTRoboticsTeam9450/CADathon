// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem INSTANCE;

  public enum ShooterPos {
    
  }
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


  public ShooterSubsystem() {

    configureShooterMotors();
    configureAngleMotor();

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

  @Override
  public void periodic() {
    if(currentState == ShooterState.SHOOTING) {
      shooterPower = velocityPID.calculate(motorWheelFront.getVelocity().getValueAsDouble());
      motorWheelBack.setVoltage(shooterPower);
      motorWheelFront.setVoltage(shooterPower);
      if(Math.abs(motorWheelFront.getVelocity().getValueAsDouble() - velocitySetpoint) < .5) {
        wheelsSpunUp = true;
      }
    }
  }

  public boolean shooterReady () {
    if(wheelsSpunUp) {
      spinUpWheels = false;
    }
    
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
