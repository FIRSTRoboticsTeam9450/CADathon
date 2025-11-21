// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem INSTANCE;

  TalonFX motorIntake = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  TalonFX motorPivot = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, RobotConstants.CANIVORE_BUS);
  PIDController PID = new PIDController(5, 0, 0);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    configIntakeMotor();
    configIntakePivot();
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
    
    // This method will be called once per scheduler run
  }

  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

}
