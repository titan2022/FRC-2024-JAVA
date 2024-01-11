package frc.robot.utility;

public class Vector2D {
    private double x;
    private double y;

    public Vector2D() {}

    public Vector2D(double inputX, double inputY) {
        throw new UnsupportedOperationException();
    }

    public double getMagnitude() {
        throw new UnsupportedOperationException();
    }

    public double add(Vector2D vec2) {
        throw new UnsupportedOperationException();
    }

    public double subtract(Vector2D vec2) {
        throw new UnsupportedOperationException();
    }
    
    /**
     * Finds the dot product of two vectors
     * @param vec2
     * @return The dot product of the two vectors
     */
    public double dot(Vector2D vec2) {
        throw new UnsupportedOperationException();
    }

    /**
     * Casts to Vector3D
     * @return Return a Vector3D with z = 0
     */
    public Vector3D get3DVector() {
        throw new UnsupportedOperationException();
    }

    /**
     * Rotates the vector theta radians counterclockwise round the unit circle
     * @param theta Angle in radians
     */
    public void rotateVector(double theta) {
        throw new UnsupportedOperationException();
    }

    /**
     * Returns a copy of the current vector
     * @return Copy of the current vector
     */
    public Vector2D copy() {
        throw new UnsupportedOperationException();
    }

    /**
     * Returns a new Vector2D using the rotateVector() and copy() methods
     * @param theta
     * @return
     */
    public Vector2D getRotatedVector(double theta) {
        throw new UnsupportedOperationException();
    }
}
