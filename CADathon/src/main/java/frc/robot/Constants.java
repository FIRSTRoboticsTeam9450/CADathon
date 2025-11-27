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

    public static class LimeLightConstants {

      public static class FrontLeft {
        public static final String NAME = "limelight-fl";
        public static final double LOCATION_X = -0.210134620269; //Offset Left/Right of robot center, -8.273 inches
        public static final double LOCATION_Y = 0.274599949199; //Offset Front/Back of robot center, 10.811 inches
        public static final double LOCATION_Z = 0.235890271781; //Offset from ground, 9.287 inches
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 37.5; //Angle pointing leftward, will need to be fixed depending on how the limelight ui deals with YAW
      }

      public static class FrontRight {
        public static final String NAME = "limelight-fr";
        public static final double LOCATION_X = 0.224993649987; //Offset Left/Right of robot center, 8.858 inches
        public static final double LOCATION_Y = 0.274777749555; //Offset Front/Back of robot center, 10.818 inches
        public static final double LOCATION_Z = 0.235890271781; //Offset from ground, 9.287 inches
        public static final double PITCH = 0;
        public static final double ROLL = 0;
        public static final double YAW = 37.5; //Angle pointing rightward, will need to be fixed depending on how the limelight ui deals with YAW
      }

    }

    /**
     * Constants Specific to the Intake Subsystem
     */
    public static class IntakeConstants {
      
      public static final int PIVOT_MOTOR_ID = 20;
      public static final int INTAKE_MOTOR_ID = 21;

    }

    /**
     * Constants specific to the Transfer Subsystem
     */
    public static class TransferConstants {

      public static final int TOWER_MOTOR_ID = 27;
      public static final int SIDE_ROLLERS_MOTOR_ID = 26;
      public static final int BOTTOM_ROLLERS_MOTOR_ID = 25;
      public static final int TOWER_CANRANGE_ID = 28;

    }

    public static class ShooterConstants {

      public static final int FRONT_WHEEL_MOTOR_ID = 30;
      public static final int BACK_WHEEL_MOTOR_ID = 31;
      public static final int ANGLE_MOTOR_ID = 32;

      public static final double FLYWHEEL_OFFSET_OF_ROBOT_CENTER = 3.572; //inches

    }
  }

}
