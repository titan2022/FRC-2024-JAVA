package frc.robot.utility;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utility.networking.NetworkingServer;

/**
 * A localizer class for the 2024 Crescendo arena
 * The global zero coordinate is in the bottom left corner as seen on pg. 24
 * Horizontal axis is positive X
 * Vertical axis is positive Y
 */
public class Localizer {
    private NetworkingServer server = new NetworkingServer();
    public WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);

    private Translation2d globalPosition = new Translation2d();
    private Rotation2d globalHeading = new Rotation2d(0);
    private Rotation2d globalOrientation = new Rotation2d(0);
    private boolean onBlueSide = false;

    public Localizer() {
        
    }

    /**
     * Gets the current global position
     */
    public Translation2d getPosition() {
        return this.globalPosition;
    }

     /**
     * Gets the current orientation estimate of the robot.
     * 
     * @return The current orientation estimate of the robot, measured
     *  counterclockwise from the positive x-axis (towards the positive y-axis).
     */
    public Rotation2d getOrientation() {
        return globalOrientation;
    }

    public double getRate() {
        return pigeon.getRate();
    }

    /**
     * Gets the current heading estimate of the robot.
     * 
     * @return The current heading estimate of the robot, measured clockwise
     *  from the positive y-axis (towards the positive x-axis).
     */
    public Rotation2d getHeading() {
        return globalHeading;
    }

    public boolean getBlueSide() {
        return onBlueSide;
    }

    public Translation2d getTagPosition(int id) {
        return new Translation2d();
    }

    public Rotation2d getTagRotation(int id) {
        return new Rotation2d();
    }

    public Rotation2d getTagHeading(int id) {
        return new Rotation2d();
    }

    public Translation2d getNotePosition() {
        return new Translation2d();
    }

    public Rotation2d getNoteRotation() {
        return new Rotation2d();
    }

    /**
     * Updates the state of the localization estimates.
     * 
     * @param dt The amount of time past since the last update, in seconds.
     */
    public synchronized void step(double dt) {
        globalHeading = pigeon.getRotation2d();
        globalOrientation = globalHeading.minus(new Rotation2d(Math.PI / 2));
    }

    /**
     * Updates the state of the localization estimates.
     * 
     * <p>This method assumes a delay of 0.02 seconds between updates. That is,
     * this method expects to be called at a frequency of 50Hz.
     */
    public synchronized void step() {
        step(0.02);
    }
}
