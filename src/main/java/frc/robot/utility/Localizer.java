package frc.robot.utility;

import java.util.Dictionary;
import java.util.Hashtable;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utility.networking.NetworkingCall;
import frc.robot.utility.networking.NetworkingServer;
import frc.robot.utility.networking.types.NetworkingPose;
import frc.robot.utility.networking.types.NetworkingTag;

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
    private NetworkingServer server;
    // private WPI_Pigeon2 pigeon = new WPI_Pigeon2(15);
    public static final AHRS navxGyro = new AHRS(SPI.Port.kMXP);

    private Rotation2d pigeonOffset = new Rotation2d(0);

    private Translation2d globalPosition = new Translation2d();
    private Rotation2d globalHeading = new Rotation2d();
    private Rotation2d globalOrientation = new Rotation2d();
    private Translation3d notePosition = new Translation3d();
    private Rotation3d noteRotation = new Rotation3d();
    private Dictionary<Integer, NetworkingTag> tags = new Hashtable<>();

    /**
     * Localizer constructor
     * 
     * @param withCoprocessor Initiate coprocessor networking server
     * @param port Coprocessor port
     */
    public Localizer(boolean withCoprocessor, int port) {
        if (withCoprocessor) {
            server = new NetworkingServer(port);
        }
    }

    /**
     * Localizer constructor
     * 
     * @param withCoprocessor Initiate coprocessor networking server with default
     *                        port
     */
    public Localizer(boolean withCoprocessor) {
        if (withCoprocessor) {
            server = new NetworkingServer();
        }
    }

    /**
     * Localizer constructor
     * 
     * @param port Coprocessor networking server specified port
     */
    public Localizer(int port) {
        this(true, port);
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
        return globalPosition;
    }

    /**
     * Gets the current orientation estimate of the robot.
     * 
     * @return The current orientation estimate of the robot, measured
     *         counterclockwise from the positive x-axis (towards the positive
     *         y-axis).
     */
    public Rotation2d getOrientation() {
        return Rotation2d.fromDegrees(-navxGyro.getAngle() + 90);
    }

    /**
     * Resets rotational offset 
     */
    public void resetOrientation() {
        navxGyro.reset();
        navxGyro.resetDisplacement();
    }

    /**
     * Gets the Pigeon rotation velocity (clockwise is positive)
     * 
     * @return The current rotational velocity (DEG/S)
     */
    public double getRate() {
        return navxGyro.getRate();
    }

    /**
     * Gets the current heading estimate of the robot.
     * 
     * @return The current heading estimate of the robot, measured clockwise
     *         from the positive y-axis
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(navxGyro.getAngle());
    }

    /**
     * Gets robot position relative to specified tag
     * 
     * @param id Apriltag ID
     * @return Top-down (XY) robot position in M
     */
    public Translation2d getTagPosition(int id) {
        return tags.get(id).position.toTranslation2d();
    }

    /**
     * Gets robot orientation relative to specified tag
     * 
     * @param id Apriltag ID
     * @return Relative top-down robot orientation from its X-axis
     */
    public Rotation2d getTagRotation(int id) {
        return tags.get(id).rotation.toRotation2d();
    }

    /**
     * Gets robot heading relative to specified tag (useful for alignment, as zero
     * is parallel)
     * 
     * @param id Apriltag ID
     * @return Relative top-down robot heading from its Y-axis
     */
    public Rotation2d getTagHeading(int id) {
        return tags.get(id).rotation.toRotation2d().minus(new Rotation2d(Math.PI / 2)).times(-1);
    }

    /**
     * Gets closest Note position to the robot's intake
     * 
     * @return Top-down (XY) robot position in M
     */
    public Translation2d getNotePosition() {
        return notePosition.toTranslation2d();
    }

    /**
     * Gets robot orientation relative to closest Note
     * 
     * @return Relative top-down robot orientation from its X-axis
     */
    public Rotation2d getNoteRotation() {
        return noteRotation.toRotation2d();
    }

    /**
     * Gets robot heading relative to closest Note (useful for alignment, as zero is
     * parallel)
     * 
     * @return Relative top-down robot heading from its Y-axis
     */
    public Rotation2d getNoteHeading() {
        return noteRotation.toRotation2d().minus(new Rotation2d(Math.PI / 2)).times(-1);
    }

    /**
     * Updates first frame localization estimates
     */
    public void setup() {
        resetOrientation();

        if (server != null) {
            server.subscribe("pos", (NetworkingCall<Translation3d>)(Translation3d position) -> {
                globalPosition = position.toTranslation2d();
            });
    
            server.subscribe("note",  (NetworkingCall<NetworkingPose>)(NetworkingPose note) -> {
                notePosition = note.position;
                noteRotation = note.rotation;
            });
    
            server.subscribe("tag", (NetworkingCall<NetworkingTag>)(NetworkingTag tag) -> {
                tags.put(tag.id, tag);
            });
        }
    }

    /**
     * Updates the state of the localization estimates
     * 
     * @param dt The amount of time past since the last update, in seconds
     */
    public synchronized void step(double dt) {
        // globalHeading = pigeon.getRotation2d().minus(pigeonOffset);
        // globalOrientation = globalHeading.minus(new Rotation2d(Math.PI / 2));

        // SmartDashboard.putNumber("acc x", navxGyro.getWorldLinearAccelX());
        // SmartDashboard.putNumber("vel x", navxGyro.getVelocityX());
        // SmartDashboard.putNumber("pos x", navxGyro.getDisplacementX());

        // SmartDashboard.putNumber("acc y", navxGyro.getWorldLinearAccelY());
        // SmartDashboard.putNumber("vel y", navxGyro.getVelocityY());
        // SmartDashboard.putNumber("pos y", navxGyro.getDisplacementY());
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
