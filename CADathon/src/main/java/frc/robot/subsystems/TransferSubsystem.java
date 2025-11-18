// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransferSubsystem extends SubsystemBase {

  TalonFX beltMotor = new TalonFX(0, "CantDrive");
  CANrange laser = new CANrange(0);
  boolean justSeen = true;
  Timer timer = new Timer();
  double timeLimit = 10;
  boolean atIntakeLimit = false;
  /** Creates a new ExampleSubsystem. */
  public TransferSubsystem() {

  }
  
  @Override
  public void periodic() {
    if(laser.getIsDetected().getValue() && justSeen) {
      justSeen = false;
      timer.start();
    }
    if(timer.get() > timeLimit) {
      atIntakeLimit
    }
    // This method will be called once per scheduler run
  }

}
