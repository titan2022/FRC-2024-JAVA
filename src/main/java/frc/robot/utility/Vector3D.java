package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation3d;

public class Vector3D extends Translation3d {
    private double x;
    private double y;
    private double z;

    public double x() { return this.x; }
    public double y() { return this.y; }
    public double z() { return this.z; }

    public Vector3D() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Vector3D(double inputX, double inputY, double inputZ) {
        this.x = inputX;
        this.y = inputY;
        this.z = inputZ;
    }

    public void setCoordinates(double inputX, double inputY, double inputZ) {
        this.x = inputX;
        this.y = inputY;
        this.z = inputZ;
    }

    public void setBearingMagnitude(double bearing, double magnitude2D, double z) {
        this.x = magnitude2D * Math.sin(bearing);
        this.y = magnitude2D * Math.cos(bearing);
        this.z = z;
    }

    public void setAngleFromEastMagnitude(double angleFromEast, double magnitude2D, double z) {
        this.x = magnitude2D * Math.cos(angleFromEast);
        this.y = magnitude2D * Math.sin(angleFromEast);
        this.z = z;
    }

    public double getMagnitude() {
        return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
    }

    public double getMagnitude2D() {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }

    /**
     * Gets the bearing of the vector, where positive y represents north.
     * @return the bearing of the vector
     */
    public double getBearing() {
        return Math.atan2(this.x, this.y);
    }

    /**
     * Gets the angle of the vector, where 0 represents positive x (east) and 
     * increasing angles represent counterclockwise.
     * @return the angle of the vector
     */
    public double getAngleFromEast() {
        return Math.atan2(this.y, this.x);
    }

    public Vector3D add(Vector3D other) {
        return new Vector3D(this.x + other.x(), this.y + other.y(), this.z + other.z());
    }

    public Vector3D subtract(Vector3D other) {
        return new Vector3D(this.x - other.x(), this.y - other.y(), this.z - other.z());
    }
    
    /**
     * Finds the dot product of two vectors
     * @param other the other vector
     * @return The dot product of the two vectors
     */
    public double dot(Vector3D other) {
        return this.x * other.x + this.y * other.y + this.z * other.z;
    }

    /**
     * Finds the angle between two vectors
     * @param other the other vector
     * @return The angle between the two vectors in radians
     */
    public double angle(Vector3D other) {
        return Math.acos(this.dot(other) / (this.getMagnitude() * other.getMagnitude()));
    }

    /**
     * Returns only the x and y components 
     * @return A vector2D with only the x and y components
     */
    public Vector2D get2DVector() {
        return new Vector2D(x, y);
    }

    public Vector3D copy() {
        return new Vector3D(x, y, z);
    }
}
