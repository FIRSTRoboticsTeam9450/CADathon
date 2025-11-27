package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class LimelightObject {
    
    public enum Location {
        FRONT_LEFT,
        FRONT_RIGHT,
        NONE
    }

    private String name;
    public double offsetX = 0;
    public double offsetY = 0;
    public double offsetZ = 0;
    public double offsetYaw = 0;
    public double offsetRoll = 0;
    public double offsetPitch = 0;
    public Location location = Location.NONE;

    public LimelightObject() {
        this("limelight");
    }

    public LimelightObject(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public LimelightObject withOffsetX(double offset) {
        this.offsetX = offset;
        return this;
    }

    public LimelightObject withOffsetY(double offset) {
        this.offsetY = offset;
        return this;
    }

    public LimelightObject withOffsetZ(double offset) {
        this.offsetZ = offset;
        return this;
    }

    public LimelightObject withOffsetYaw(double offset) {
        this.offsetYaw = offset;
        return this;
    }

    public LimelightObject withOffsetRoll(double offset) {
        this.offsetRoll = offset;
        return this;
    }

    public LimelightObject withOffsetPitch(double offset) {
        this.offsetPitch = offset;
        return this;
    }

    public Translation3d getTranslationOffset() {
        return new Translation3d(
                this.offsetX, 
                this.offsetY, 
                this.offsetZ
            );
    }

    public Rotation3d getRotationOFfset() {
        return new Rotation3d(
                this.offsetRoll,
                this.offsetPitch,
                this.offsetYaw
            );
    }

    public Translation2d getTranslationToRobotCenter() {
        return new Translation2d(
                this.offsetX,
                this.offsetY
            );
    }

}
