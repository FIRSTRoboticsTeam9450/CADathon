// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Constants relating to the robot
   */
  public final class RobotConstants {

    /* ---------- CAN Bus Names --------- */
    public static final String CANIVORE_BUS = "CantDrive";
    public static final String RIO_BUS = "Rio";

    /* ---------- Default Neutral Mode for Motors ---------- */
    public static final NeutralModeValue DEFAULT_NEUTRAL_MODE = NeutralModeValue.Brake;

    /**
     * Constants Specific to the Intake Subsystem
     */
    public static class IntakeConstants {
      
      public static final int PIVOT_MOTOR_ID = 0;
      public static final int INTAKE_MOTOR_ID = 0;

    }

    /**
     * Constants specific to the Transfer Subsystem
     */
    public static class TransferConstants {

      public static final int TOWER_MOTOR_ID = 0;
      public static final int SIDE_ROLLERS_MOTOR_ID = 0;
      public static final int BOTTOM_ROLLERS_MOTOR_ID = 0;
      public static final int TOWER_CANRANGE_ID = 0;

    }
  }

}
