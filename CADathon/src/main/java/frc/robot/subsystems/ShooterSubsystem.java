// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private static ShooterSubsystem INSTANCE;

  private TalonFX motorWheelFront = new TalonFX(ShooterConstants.FRONT_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorWheelBack = new TalonFX(ShooterConstants.BACK_WHEEL_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorAngle = new TalonFX(ShooterConstants.ANGLE_MOTOR_ID, RobotConstants.CANIVORE_BUS);

  private double velocity = 2;
  private double acceleration = 100;
  private double jerk = 1000;
  private double currentLimit = 60; // 100 is the max stator current pull
  private double kS = 0.6; // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
  private double kV = 0.26; // A velocity target of 1 rps results in 0.12 V output .12
  private double kA = 0.017; // An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
  private double kP = 3; // A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
  private double kI = 0; // no output for integrated error 0
  private double kD = 0.12; // A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
  private double kG = 0.45;
  private DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);

  private boolean isZeroingDone;

  public ShooterSubsystem() {

    configureShooterMotors();
    configureAngleMotor();

    isZeroingDone = false;

  }

  private void configureShooterMotors() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorWheelFront.getConfigurator().apply(motorConfig);
    motorWheelBack.getConfigurator().apply(motorConfig);
  }

  private void configureAngleMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = RobotConstants.DEFAULT_NEUTRAL_MODE;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
    slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
    slot0.kG = kG;

    motorConfig.Slot0 = slot0;

    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
    motorConfig.MotionMagic.MotionMagicJerk = jerk;

    motorAngle.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
      if (!isZeroingDone) {
        isZeroingDone = zeroEncoder();
      }
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


  public static ShooterSubsystem getInsatnce() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  } 

}
