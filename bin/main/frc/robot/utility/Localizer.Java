package frc.robot.subsystems;

/**
 * The april tags will be used to redefine the global positions of the robot
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf
 * Page 37 talks about the IDs of the april tags 
 * If you wish you can define an enumeration for the apriltags IDs that the camera will send to the roboRio
 * for the sake of convienience 
 */
public enum AprilTags {

}

/**
 * A localizer class for the 2024 Crescendo arena
 * The rule book on pg 22 has an image of the arena
 * https://firstfrc.blob.core.windows.net/frc2024/Manual/2024GameManual.pdf
 * Consider the bottom left corner of the source area(pg 24) as the origin
 * X is positive going from left to right
 * Y is position going from bottom to top
 * The global orientation of 0 will be considered along the global x axis from left to right
 * Basically, imagine the field as a unit circle and you should be good
 */
public class Localizer {

    public Vector2D globalPosition;
    public Vector2D localPosition;
    public double globalOrientation;
    public double localOrienation;

    public Localizer(double inputX, double inputY, double inputlocalOrientation. double inputLocalOrientation) {
        throw new UnsupportedOperationException();
    }

    /**
     * Sets the local position to a new position
     */
    public void setLocalPosition(Vector2D vec) {
        throw new UnsupportedOperationException();
    }

    /**
     * Gets the current local position
     */
    public Vector2D getLocalPosition() {
        throw new UnsupportedOperationException();
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateLocalPosition(Vector2D vec) {
        throw new UnsupportedOperationException();
    }

    
    /**
     * Sets the local position to a new position
     */
    public void setGlobalPosition(Vector2D vec) {
        throw new UnsupportedOperationException();
    }

    /**
     * Gets the current local position
     */
    public Vector2D getGlobalPosition() {
        throw new UnsupportedOperationException();
    }

    /**
     * Adds the vector to the current local position to update it
     */
    public void updateGlobalPosition(Vector2D vec) {
        throw new UnsupportedOperationException();
    }

    /**
     * Sets the local orientation to a new orientation
     */
    public void setLocalOrientation(double theta) {
        throw new UnsupportedOperationException();
    }

    /**
     * Gets the current local orientation
     */
    public double getLocalOrientation() {
        throw new UnsupportedOperationException();
    }

    /**
     * Adds the angle to the current local orientation to update it
     */
    public void updateLocalOrientation(double theta) {
        throw new UnsupportedOperationException();
    }

    /**
     * Sets the global orientation to a new orientation
     */
    public void setGlobalOrientation(double theta) {
        throw new UnsupportedOperationException();
    }

    /**
     * Gets the current global orientation
     */
    public double getGlobalOrientation() {
        throw new UnsupportedOperationException();
    }

    /**
     * Adds the angle to the current global orientation to update it
     */
    public void updateGlobalOrientation(double theta) {
        throw new UnsupportedOperationException();
    }

    /**
     * Updates the global position and orientation of the robot based upon the detected april tag
     * Decide whether you will use the IDs specified in rule book via int or using the enum AprilTag
     */
    public void updateAprilTag(AprilTag tag, Vector3D distance, double angle) {

    }

}