package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class Vector2D extends Translation2d {
    private double x;
    private double y;

    public double x() {
        return this.x;
    }

    public double y() {
        return this.y;
    }

    public Vector2D() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2D(double inputX, double inputY) {
        this.x = inputX;
        this.y = inputY;
    }

    public Vector2D(Translation2d input) {
        super(input.getX(), input.getY());
    }

    public void setCoordinates(double inputX, double inputY) {
        this.x = inputX;
        this.y = inputY;
    }

    public void setBearingMagnitude(double bearing, double magnitude) {
        this.x = magnitude * Math.sin(bearing);
        this.y = magnitude * Math.cos(bearing);
    }

    public void setAngleFromEastMagnitude(double angleFromEast, double magnitude) {
        this.x = magnitude * Math.cos(angleFromEast);
        this.y = magnitude * Math.sin(angleFromEast);
    }

    public double getMagnitude() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public double getMagnitude2D() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    /**
     * Gets the bearing of the vector, where positive y represents north.
     * 
     * @return the bearing of the vector
     */
    public double getBearing() {
        return Math.atan2(this.x, this.y);
    }

    /**
     * Gets the angle of the vector, where 0 represents positive x (east) and
     * increasing angles represent counterclockwise.
     * 
     * @return the angle of the vector
     */
    public double getAngleFromEast() {
        return Math.atan2(this.y, this.x);
    }

    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.x(), this.y + other.y());
    }

    public Vector2D subtract(Vector2D other) {
        return new Vector2D(this.x - other.x(), this.y - other.y());
    }

    /**
     * Finds the dot product of two vectors
     * 
     * @param other the other vector
     * @return The dot product of the two vectors
     */
    public double dot(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Finds the angle between two vectors
     * 
     * @param other the other vector
     * @return The angle between the two vectors in radians
     */
    public double angle(Vector2D other) {
        return Math.acos(this.dot(other) / (this.getMagnitude2D() * other.getMagnitude2D()));
    }

    /**
     * Casts to Vector3D
     * 
     * @return Return a Vector3D with z = 0
     */
    public Vector3D get3DVector() {
        return new Vector3D(x, y, 0);
    }

    /**
     * Rotates the vector theta radians counterclockwise round the unit circle
     * 
     * @param theta Angle in radians
     */
    public void rotateVector(double theta) {
        this.setAngleFromEastMagnitude(this.getAngleFromEast() + theta, this.getMagnitude());
    }

    /**
     * Returns a copy of the current vector
     * 
     * @return Copy of the current vector
     */
    public Vector2D copy() {
        return new Vector2D(x, y);
    }

    /**
     * Returns a new Vector2D using the rotateVector() and copy() methods
     * 
     * @param theta
     * @return
     */
    public Vector2D getRotatedVector(double theta) {
        Vector2D retval = this.copy();
        retval.rotateVector(theta);
        return retval;
    }
}
