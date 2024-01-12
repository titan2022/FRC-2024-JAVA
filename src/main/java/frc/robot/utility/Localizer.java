package frc.robot.utility;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A localizer class for the 2024 Crescendo arena
 * The rule book on pg 22 has an image of the arena
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf
 * Consider the bottom left corner of the source area(pg 24) as the origin
 * X is positive going from left to right
 * Y is position going from bottom to top
 * The global orientation of 0 will be considered along the global x axis from
 * left to right
 * Basically, imagine the field as a unit circle and you should be good
 */

public class Localizer {

    public Translation2d globalPosition = new Translation2d();
    public Translation2d localPosition = new Translation2d();
    public double globalOrientation = 0;
    public double localOrientation = 0;
    public boolean on_blue_side = false;

    public Localizer(double inputX, double inputY, double inputlocalOrientation) {
        localPosition = new Translation2d(inputX, inputY);
        localOrientation = inputlocalOrientation;
    }

    /**
     * Sets the local position to a new position
     */
    public void setLocalPosition(Translation2d vec) {
        this.localPosition = vec;
    }

    /**
     * Gets the current local position
     */
    public Translation2d getLocalPosition() {
        return this.localPosition;
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateLocalPosition(Translation2d vec) {
        this.localPosition = this.localPosition.plus(vec);
    }

    /**
     * Sets the local position to a new position
     */
    public void setGlobalPosition(Translation2d vec) {
        this.globalPosition = vec;
    }

    /**
     * Gets the current local position
     */
    public Translation2d getGlobalPosition() {
        return this.globalPosition;
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateGlobalPosition(Translation2d vec) {
        this.globalPosition = this.globalPosition.plus(vec);
    }

    /**
     * Sets the local orientation to a new orientation
     */
    public void setLocalOrientation(double theta) {
        this.localOrientation = theta;
    }

    /**
     * Gets the current local orientation
     */
    public double getLocalOrientation() {
        return this.localOrientation;
    }

    /**
     * Adds the angle to the current local orientation to update it
     */
    public void updateLocalOrientation(double theta) {
        this.localOrientation += theta;
    }

    /**
     * Sets the global orientation to a new orientation
     */
    public void setGlobalOrientation(double theta) {
        this.globalOrientation = theta;
    }

    /**
     * Gets the current global orientation
     */
    public double getGlobalOrientation() {
        return this.globalOrientation;
    }

    /**
     * Adds the angle to the current global orientation to update it
     */
    public void updateGlobalOrientation(double theta) {
        this.globalOrientation += theta;
    }

    /**
     * Updates the global position and orientation of the robot based upon the
     * detected april tag
     * Decide whether you will use the IDs specified in rule book via int or using
     * the enum AprilTag
     */
    // public void updateWithAprilTag(AprilTag tag, Vector3D relative_position, double angle) {

    // }

}
