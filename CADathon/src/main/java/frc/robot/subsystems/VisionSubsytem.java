package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontLeft;
import frc.robot.Constants.RobotConstants.LimeLightConstants.FrontRight;
import frc.robot.util.LimelightObject;
import frc.robot.util.LimelightObject.Location;

public class VisionSubsytem extends SubsystemBase {

    private static VisionSubsytem INSTANCE;

    private LimelightObject limelightFL;
    private LimelightObject limelightFR;

    private void configureLimelights() {
        limelightFL = new LimelightObject(FrontLeft.NAME);
        limelightFL.location = Location.FRONT_LEFT;
        limelightFL.withOffsetX(FrontLeft.LOCATION_X)
                    .withOffsetY(FrontLeft.LOCATION_Y)
                    .withOffsetZ(FrontLeft.LOCATION_Z)
                    .withOffsetPitch(FrontLeft.PITCH)
                    .withOffsetRoll(FrontLeft.ROLL)
                    .withOffsetYaw(FrontLeft.YAW);

        limelightFR = new LimelightObject(FrontRight.NAME);
        limelightFR.location = Location.FRONT_RIGHT;
        limelightFR.withOffsetX(FrontRight.LOCATION_X)
                    .withOffsetY(FrontRight.LOCATION_Y)
                    .withOffsetZ(FrontRight.LOCATION_Z)
                    .withOffsetPitch(FrontRight.PITCH)
                    .withOffsetRoll(FrontRight.ROLL)
                    .withOffsetYaw(FrontRight.YAW);

    }



    public static VisionSubsytem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsytem();
        }
        return INSTANCE;
    }
    
}
