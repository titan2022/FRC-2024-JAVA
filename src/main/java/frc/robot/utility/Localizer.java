package frc.robot.utility;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utility.networking.NetworkingServer;

/**
 * Localizer class for the 2024 Crescendo arena
 * <p>
 * The global zero coordinate is in the bottom left corner as seen on Pg. 24.
 * Horizontal axis is positive X, vertical axis is positive Y
 * <p>
 * Positive Y and positive X are robot's front and right sides respectively in
 * robot-relative coorditate plane
 */
public class Localizer {
    private NetworkingServer server = new NetworkingServer();
    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);

    private Rotation2d pigeonOffset = new Rotation2d(0);
    private Translation2d globalPosition = new Translation2d();
    private Rotation2d globalHeading = new Rotation2d(0);
    private Rotation2d globalOrientation = new Rotation2d(0);
    private boolean onBlueSide = false;

    /**
     * Localizer constructor
     * 
     * @param withCoprocessor Initiate coprocessor networking server
     */
    public Localizer(boolean withCoprocessor) {

    }

    /**
     * Localizer constructor (initiates coprocessor networking server by default)
     */
    public Localizer() {
        this(true);
    }

    /**
     * Gets the current global position
     * 
     * @return Position in meters from bottom left corner
     */
    public Translation2d getPosition() {
        return this.globalPosition;
    }

    /**
     * Gets the current orientation estimate of the robot.
     * 
     * @return The current orientation estimate of the robot, measured
     *         counterclockwise from the positive x-axis (towards the positive
     *         y-axis).
     */
    public Rotation2d getOrientation() {
        return globalOrientation;
    }

    /**
     * Gets the Pigeon rotation velocity (clockwise is positive)
     * 
     * @return The current rotational velocity (DEG/S)
     */
    public double getRate() {
        return pigeon.getRate();
    }

    /**
     * Gets the current heading estimate of the robot.
     * 
     * @return The current heading estimate of the robot, measured counterclockwise
     *         from the positive y-axis
     */
    public Rotation2d getHeading() {
        return globalHeading;
    }

    /**
     * Determines if robot is on blue alliance's side of the field
     * 
     * @return Boolean whether the robot is on blue side
     */
    public boolean getBlueSide() {
        return onBlueSide;
    }

    /**
     * Gets robot position relative to specified tag
     * 
     * @param id Apriltag ID
     * @return Top-down (XY) robot position in M
     */
    public Translation2d getTagPosition(int id) {
        return new Translation2d();
    }

    /**
     * Gets robot orientation relative to specified tag
     * 
     * @param id Apriltag ID
     * @return Relative top-down robot orientation from its X-axis
     */
    public Rotation2d getTagRotation(int id) {
        return new Rotation2d();
    }

    /**
     * Gets robot heading relative to specified tag (useful for alignment, as zero
     * is parallel)
     * 
     * @param id Apriltag ID
     * @return Relative top-down robot heading from its Y-axis
     */
    public Rotation2d getTagHeading(int id) {
        return new Rotation2d();
    }

    /**
     * Gets closest Note position to the robot's intake
     * 
     * @return Top-down (XY) robot position in M
     */
    public Translation2d getNotePosition() {
        return new Translation2d();
    }

    /**
     * Gets robot orientation relative to closest Note
     * 
     * @return Relative top-down robot orientation from its X-axis
     */
    public Rotation2d getNoteRotation() {
        return new Rotation2d();
    }

    /**
     * Gets robot heading relative to closest Note (useful for alignment, as zero is
     * parallel)
     * 
     * @return Relative top-down robot heading from its Y-axis
     */
    public Rotation2d getNoteHeading() {
        return new Rotation2d();
    }

    /**
     * Updates first frame localization estimates
     */
    public synchronized void setup() {
        pigeonOffset = pigeon.getRotation2d();
    }

    /**
     * Updates the state of the localization estimates
     * 
     * @param dt The amount of time past since the last update, in seconds
     */
    public synchronized void step(double dt) {
        globalHeading = pigeon.getRotation2d().minus(pigeonOffset);
        globalOrientation = globalHeading.minus(new Rotation2d(Math.PI / 2));
    }

    /**
     * Updates the state of the localization estimates
     * 
     * <p>
     * This method assumes a delay of 0.02 seconds between updates. That is,
     * this method expects to be called at a frequency of 50Hz
     */
    public synchronized void step() {
        step(0.02);
    }
}
