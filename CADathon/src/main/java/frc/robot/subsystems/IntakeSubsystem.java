// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem INSTANCE;

  public enum intakePos {
    SPEECH_BUBBLES_INTAKE,
    STORY_BOARDS_INTAKE,
    STORE,
    ZERO,
    STORY_BOARDS_SCORE
  }

  public enum intakeStates {
    INTAKING,
    OUTTAKING,
    NOT_RUNNING
  }

  private TalonFX motorIntake = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  private TalonFX motorPivot = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, RobotConstants.CANIVORE_BUS); // Good voltage is around 1.5V

  private LoggedNetworkNumber logMMVel = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Velocity", 9);
  private LoggedNetworkNumber logMMAccl = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Acceleration", 11);
  private LoggedNetworkNumber logMMJerk = new LoggedNetworkNumber("/Tuning/Intake/Pivot/Jerk", 1000);

  private double velocity = logMMVel.get();
  private double acceleration = logMMAccl.get();
  private double jerk = logMMJerk.get();

  private LoggedNetworkNumber logMMKS = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kS", 0);
  private LoggedNetworkNumber logMMKV = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kV", 0);
  private LoggedNetworkNumber logMMKA = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kA", 0);
  private LoggedNetworkNumber logMMKP = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kP", 15);
  private LoggedNetworkNumber logMMKI = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kI", 0);
  private LoggedNetworkNumber logMMKD = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kD", 0);
  private LoggedNetworkNumber logMMKG = new LoggedNetworkNumber("/Tuning/Intake/Pivot/kG", 0);

  private double kS = logMMKS.get(); // Add 0.25 V output to overcome static friction .25 - Gives it a little boost in the very beginning
  private double kV = logMMKV.get(); // A velocity target of 1 rps results in 0.12 V output .12
  private double kA = logMMKA.get(); // An acceleration of 1 rps/s requires 0.01 V output .01 - Adds a little boost
  private double kP = logMMKP.get(); // A position error of 2.5 rotations results in 12 V output 3.8 - Helps correct positional error
  private double kI = logMMKI.get(); // no output for integrated error 0
  private double kD = logMMKD.get(); // A velocity error of 1 rps results in 0.1 V output 0.1 - Can help correct kV and kA error
  private double kG = logMMKG.get();
  
  private DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);

  private intakePos currentPos = intakePos.ZERO;
  private intakePos prevPos = intakePos.ZERO;
  private intakePos targetPos = intakePos.ZERO;

  private intakeStates currState = intakeStates.NOT_RUNNING;
  private intakeStates targetState = intakeStates.NOT_RUNNING;

  private boolean resetDone = false;
  private double setpoint;

  public IntakeSubsystem() {

    configIntakeMotor();
    configIntakePivot();
    motorPivot.setPosition(0);

  }
  
  public void configIntakeMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;

    motorIntake.getConfigurator().apply(motorConfig);
  }
  
  public void configIntakePivot() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.8;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 25;
    
    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
    slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
    slot0.kG = kG;

    motorConfig.Slot0 = slot0;

    motorConfig.MotionMagic.MotionMagicAcceleration = acceleration;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = velocity;
    motorConfig.MotionMagic.MotionMagicJerk = jerk;

    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 8.85;

    motorPivot.getConfigurator().apply(motorConfig);
  }

  private void updatePivotConstants() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    Slot0Configs slot0 = motorConfig.Slot0;
    slot0.kS = kS; slot0.kV = kV; slot0.kA = kA;
    slot0.kP = kP; slot0.kI = kI; slot0.kD = kD;
    slot0.kG = kG;

    motorConfig.Slot0 = slot0;

    motorPivot.getConfigurator().apply(motorConfig);
  }
  
  @Override
  public void periodic() {
    // Zero encoder
    // if(!resetDone) {
    //   resetDone = isEncoderReset();
    // }

    if(targetPos != currentPos) {
      goToPos();
    }
    if(currState != targetState) {
      setIntakeVoltage(getIntakeVoltage(targetState));
    }
    motorPivot.setControl(request.withPosition(setpoint));
    
    publishLogs();

  }

  private void publishLogs() {
    Logger.recordOutput("HeroHeist/IntakeSubsystem/TargetPos", targetPos);
    Logger.recordOutput("HeroHeist/IntakeSubsystem/MotorPos", motorPivot.getPosition().getValueAsDouble());
    Logger.recordOutput("HeroHeist/IntakeSubsystem/setpoint", setpoint);
    Logger.recordOutput("HeroHeist/IntakeSubsystem/velocity", motorPivot.getVelocity().getValueAsDouble());
  }

  public boolean atSetpoint() {
    return Math.abs(getPosition(currentPos) - getPosition(targetPos)) < .1;
  }

  public void setTargetPos(intakePos targetPos, intakeStates targetState) {
    this.targetPos = targetPos;
    this.targetState = targetState;
  }

  public void goToPos() {
    prevPos = currentPos;
    if(prevPos == intakePos.STORE || prevPos == intakePos.STORY_BOARDS_SCORE) {
      setSetpoint(getPosition(targetPos));
      currentPos = targetPos;
    }
    else {
      setSetpoint(getPosition(intakePos.STORE));
      currentPos = intakePos.STORE;
    }
  }

  private boolean isEncoderReset() {
    if(Math.abs(motorPivot.getVelocity().getValueAsDouble()) < .1) {
      motorPivot.setPosition(0);
      motorPivot.setVoltage(0);
      setTargetPos(intakePos.STORE, intakeStates.NOT_RUNNING);
      return true;
    }
    return false;
  }

  public void zeroEncoder() {
    if (DriverStation.isDisabled()) {
      motorPivot.setPosition(0);
    }
  }

  public void updateConstants() {
    boolean updatePivotVals = false;
    boolean updateMMRequest = false;

    double lMMKSVal = logMMKS.get();
    double lMMKVVal = logMMKV.get();
    double lMMKAVal = logMMKA.get();
    double lMMKPVal = logMMKP.get();
    double lMMKIVal = logMMKI.get();
    double lMMKDVal = logMMKD.get();
    double lMMKGVal = logMMKG.get();

    updatePivotVals = (kS != lMMKSVal)
                   || (kV != lMMKVVal)
                   || (kA != lMMKAVal)
                   || (kP != lMMKPVal)
                   || (kI != lMMKIVal)
                   || (kD != lMMKDVal)
                   || (kG != lMMKGVal);

    if (updatePivotVals) {
      kS = lMMKSVal;
      kV = lMMKVVal;
      kA = lMMKAVal;
      kP = lMMKPVal;
      kI = lMMKIVal;
      kD = lMMKDVal;
      kG = lMMKGVal;

      updatePivotConstants();
    }

    updateMMRequest = (velocity != logMMVel.get()) 
                   || (acceleration != logMMAccl.get()) 
                   || (jerk != logMMJerk.get());

    if (updateMMRequest) {
      velocity = logMMVel.get();
      acceleration = logMMAccl.get();
      jerk = logMMJerk.get();

      request = new DynamicMotionMagicVoltage(0, velocity, acceleration, jerk);
    }
  }

  /*** ____________________________________ SETTERS ____________________________________ ***/

  public void setIntakeVoltage(double voltage) {
    if(atSetpoint()) {
      currState = targetState;
      motorIntake.setVoltage(voltage);
    }
  }

  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }
  
  /*** ____________________________________ GETTERS ____________________________________ ***/

  public double getPosition(intakePos pos) {
    switch (pos) {
      case STORE:
        return 4.5;
      case SPEECH_BUBBLES_INTAKE:
        return 0.775;
      case STORY_BOARDS_INTAKE:
        return 0;
      case STORY_BOARDS_SCORE:
        return 0;
      default:
        return 0;
    }
  }

  public double getIntakeVoltage(intakeStates state) {
    switch (state) {
      case INTAKING:
        return 6;
      case OUTTAKING:
        return 0;
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
