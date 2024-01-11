package frc.robot.commands;

import frc.robot.utility.FieldObject;
import frc.robot.utility.Vector2D;

/**
 * The april tags will be used to redefine the global positions of the robot
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf#page=37
 * Page 37 talks about the IDs of the april tags 
 * If you wish you can define an enumeration for the apriltags IDs that the camera will send to the roboRio
 * for the sake of convienience 
 */


public enum AprilTag implements FieldObject {
    // SOUTH is towards the SCORING TABLE
    // EAST is towards the Red Alliance Area
    // WEST is towards the Blue Alliance Area
    BLUE_SOURCE_SOUTH(1), BLUE_SOURCE_NORTH(2),
    RED_SOURCE_SOUTH(10), RED_SOURCE_NORTH(9),
    RED_SPEAKER_SOUTH(3), RED_SPEAKER_CENTER(4),
    BLUE_SPEAKER_SOUTH(8), BLUE_SPEAKER_NORTH(7),
    RED_AMP(5), BLUE_AMP(6),
    RED_STAGE_SOUTH(11), RED_STAGE_NORTH(12), RED_STAGE_WEST(13),
    BLUE_STAGE_SOUTH(16), BLUE_STAGE_NORTH(15), BLUE_STAGE_EAST(14);

    private int id;
    private String name;
    private Vector2D position;

    private AprilTag(int id) {
        this.id = id;
    }

    public void setName(String name) {
        this.name = name;
    }

    @Override
    public String getName() {
        return this.name;
    }

    public void setPosition(Vector2D position) {
        this.position = position;
    }

    @Override
    public Vector2D getPosition() {
        return this.position;
    }
}
