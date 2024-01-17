package frc.robot.utility.localization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The april tags will be used to redefine the global positions of the robot
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf#page=37
 * Page 37 talks about the IDs of the april tags
 * If you wish you can define an enumeration for the apriltags IDs that the
 * camera will send to the roboRio
 * for the sake of convienience
 */

public enum AprilTag implements FieldObject {
    // SOUTH is towards the SCORING TABLE
    // EAST is towards the Red Alliance Area
    // WEST is towards the Blue Alliance Area
    // 1 in = 0.0254 m
    // See https://docs.google.com/spreadsheets/d/1aTBUTqv_QWHWGeNzhRpJv8-SgyZ2aXf8bqGZQpopXcw/edit?usp=sharing
    BLUE_SOURCE_SOUTH(1, new Translation3d(15.079472, 0.245872, 1.355852), new Rotation2d(2.0943951023932)), 
    BLUE_SOURCE_NORTH(2, new Translation3d(16.185134, 0.883666, 1.355852), new Rotation2d(2.0943951023932)),
    RED_SOURCE_SOUTH(10, new Translation3d(1.461516, 0.245872, 1.355852), new Rotation2d(1.0471975511966)), 
    RED_SOURCE_NORTH(9, new Translation3d(0.356108, 0.883666, 1.355852), new Rotation2d(1.0471975511966)),
    RED_SPEAKER_SOUTH(3, new Translation3d(16.579342, 4.982718, 1.451102), new Rotation2d(3.14159265358979)), 
    RED_SPEAKER_CENTER(4, new Translation3d(16.579342, 5.547868, 1.451102), new Rotation2d(3.14159265358979)),
    BLUE_SPEAKER_SOUTH(8, new Translation3d(-0.0381, 4.982718, 1.451102), new Rotation2d(0)), 
    BLUE_SPEAKER_NORTH(7, new Translation3d(-0.0381, 5.547868, 1.451102), new Rotation2d(0)),
    RED_AMP(5, new Translation3d(14.700758, 8.2042, 1.355852), new Rotation2d(4.71238898038469)), 
    BLUE_AMP(6, new Translation3d(1.8415, 8.2042, 1.355852), new Rotation2d(4.71238898038469)),
    RED_STAGE_SOUTH(11, new Translation3d(11.904726, 3.713226, 1.3208), new Rotation2d(5.23598775598299)), 
    RED_STAGE_NORTH(12, new Translation3d(11.904726, 4.49834, 1.3208), new Rotation2d(1.0471975511966)), 
    RED_STAGE_WEST(13, new Translation3d(11.220196, 4.105148, 1.3208), new Rotation2d(3.14159265358979)),
    BLUE_STAGE_SOUTH(16, new Translation3d(4.641342, 3.713226, 1.3208), new Rotation2d(4.18879020478639)), 
    BLUE_STAGE_NORTH(15, new Translation3d(4.641342, 4.49834, 1.3208), new Rotation2d(2.0943951023932)), 
    BLUE_STAGE_EAST(14, new Translation3d(5.320792, 4.105148, 1.3208), new Rotation2d(0)),
    ;

    private int id;
    private String name;
    private Translation3d position;
    private Rotation2d normalOrientation;

    private AprilTag(int id, Translation3d position, Rotation2d normalOrientation) {
        this.id = id;
        this.position = position;
        this.normalOrientation = normalOrientation;
    }

    public FieldObjectType getType() {
        return FieldObjectType.AprilTag;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return this.name;
    }

    public void setPosition(Translation2d position) {
        this.position = new Translation3d(position.getX(), position.getY(), this.position.getZ());
    }

    public void setPosition3d(Translation3d position) {
        this.position = position;
    }

    @Override
    public Translation2d getPosition() {
        return this.position.toTranslation2d();
    }

    public Translation3d getPosition3d() {
        return this.position;
    }

    public int getID() {
        return this.id;
    }

    public Rotation2d getNormalOrientation() {
        return this.normalOrientation;
    }
}
