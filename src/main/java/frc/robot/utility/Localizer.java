package frc.robot.utility;

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

    public Vector2D globalPosition;
    public Vector2D localPosition;
    public double globalOrientation;
    public double localOrientation;
    public boolean on_blue_side;

    public Localizer(double inputX, double inputY, double inputlocalOrientation, double inputLocalOrientation) {
        throw new UnsupportedOperationException();
    }

    /**
     * Sets the local position to a new position
     */
    public void setLocalPosition(Vector2D vec) {
        this.localPosition = vec;
    }

    /**
     * Gets the current local position
     */
    public Vector2D getLocalPosition() {
        return this.localPosition;
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateLocalPosition(Vector2D vec) {
        this.localPosition = this.localPosition.add(vec);
    }

    /**
     * Sets the local position to a new position
     */
    public void setGlobalPosition(Vector2D vec) {
        this.globalPosition = vec;
    }

    /**
     * Gets the current local position
     */
    public Vector2D getGlobalPosition() {
        return this.globalPosition;
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateGlobalPosition(Vector2D vec) {
        this.globalPosition = this.globalPosition.add(vec);
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
    public void updateWithAprilTag(AprilTag tag, Vector3D relative_position, double angle) {

    }

}