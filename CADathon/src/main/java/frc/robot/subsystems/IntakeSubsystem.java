// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem INSTANCE;

  public enum intakePos {
    SPEECH_BUBBLES_INTAKE,
    STORY_BOARDS_INTAKE,
    STORE
  }

  public enum intakeState {
    INTAKING,
    OUTTAKING,
    NOT_RUNNING
  }

  TalonFX motorIntake = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  TalonFX motorPivot = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  PIDController PID = new PIDController(5, 0, 0);

  double velocity = 2;
  double acceleration = 100;
  double jerk = 1000;
  double currentLimit = 130; // 100 is the max stator current pull
  double kS = 0.6; // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
  double kV = 0.26; // A velocity target of 1 rps results in 0.12 V output .12
  double kA = 0.017; // An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
  double kP = 3; // A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
  double kI = 0; // no output for integrated error 0
  double kD = 0.12; // A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
  double kG = 0.45;
  DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);

  intakePos currentPos = intakePos.STORE;

  boolean resetDone = false;
  
  
  public IntakeSubsystem() {
    configIntakeMotor();
    configIntakePivot();
    motorPivot.setVoltage(-1);
  }
  
  public void configIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorIntake.getConfigurator().apply(config);
  }
  
  public void configIntakePivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorPivot.getConfigurator().apply(config);
  }
  
  @Override
  public void periodic() {


    // Zero encoder
    if(!resetDone) {
      resetDone = isEncoderReset();
    }
    
  }

  public void goToPos(intakePos targetPos, intakeState state) {
    if(currentPos == intakePos.STORE) {
      setSetpoint(getPosition(targetPos));
      setIntakeVoltage(getIntakeVoltage(state));
    }
    else {
      setSetpoint(getPosition(intakePos.STORE));
    }
    
  }

  public boolean isEncoderReset() {
    if(motorPivot.getVelocity().getValueAsDouble() < .1) {
      motorPivot.setPosition(0);
      motorPivot.setVoltage(0);
      return true;
    }
    return false;
  }

  /*** ____________________________________ SETTERS ____________________________________ ***/

  public void setIntakeVoltage(double voltage) {
    motorIntake.setVoltage(voltage);
  }

  public void setSetpoint(double setpoint) {
    PID.setSetpoint(setpoint);
  }
  
  /*** ____________________________________ GETTERS ____________________________________ ***/

  public double getPosition(intakePos pos) {
    switch (pos) {
      case STORE:
        return 0;
      case SPEECH_BUBBLES_INTAKE:
        return 0;
      case STORY_BOARDS_INTAKE:
        return 0;
      default:
        return 0;
    }
  }

  public double getIntakeVoltage(intakeState state) {
    switch (state) {
      case INTAKING:
        return 6;
      case OUTTAKING:
        return -6;
      case NOT_RUNNING:
        return 0;
      default:
        return 0;
    }
  }
  
  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

}
