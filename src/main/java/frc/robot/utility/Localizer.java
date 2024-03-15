package frc.robot.utility;

import java.util.Dictionary;
import java.util.Hashtable;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
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
    private PhotonCamera camera = new PhotonCamera("photonvision");
    private NetworkingServer server;
    private SwerveDriveSubsystem drive;
    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(15);
    public static final AHRS navxGyro = new AHRS(SPI.Port.kMXP);

    private DoubleLogEntry xLog;
    private DoubleLogEntry yLog;


    private Rotation2d pigeonOffset = new Rotation2d(0);

    private Translation2d globalPosition = new Translation2d();
    private Rotation2d globalHeading = new Rotation2d();
    private Rotation2d globalOrientation = new Rotation2d();
    private Rotation3d globalOrientationFromTags = new Rotation3d();
    private double noteDistance = -1;
    private Rotation3d noteRotation = new Rotation3d();
    private double speakerDist;
    private Rotation2d speakerHeading;
    private Dictionary<Integer, NetworkingTag> tags = new Hashtable<>();

    private Pose2d startingPose2d = new Pose2d(); 
    /**
     * Localizer constructor
     * 
     * @param withCoprocessor Initiate coprocessor networking server
     * @param port Coprocessor port
     */
    public Localizer(SwerveDriveSubsystem drive, boolean withCoprocessor, int port) {
        this.drive = drive;
        if (withCoprocessor) {
            server = new NetworkingServer(port);
        }
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
        // return Rotation2d.fromDegrees(-navxGyro.getAngle() + 90);
        return globalOrientation;
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
        // return navxGyro.getRate();
        return pigeon.getRate();
    }

    /**
     * Gets the current heading estimate of the robot.
     * 
     * @return The current heading estimate of the robot, measured clockwise
     *         from the positive y-axis
     */
    public Rotation2d getHeading() {
        // return Rotation2d.fromDegrees(navxGyro.getAngle());
        return globalHeading;
    }

    /**
     * Gets closest Note distance to the robot's intake
     * 
     * @return Top-down (XY) robot distance in M
     */
    public double getNoteDistance() {
        return noteDistance;
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

    public void resetStartingPose2d(Pose2d pose){
        startingPose2d=pose;
    }
    /**
     * Gets Pose2d based on translation and rotation for path planner
     * @return
     */
    public Pose2d getDisplacementPose2d() {
        return (new Pose2d(globalPosition,globalOrientation)).relativeTo(startingPose2d);
    }

    /**
     * Gets global rotation of speaker
     * @return Rotation2d
     */
    public Rotation2d getSpeakerHeading() {
        return speakerHeading;
    }

    /**
     * Gets distance from shooter to speaker in meters
     * @return meters
     */
    public double getSpeakerDistance() {
        return speakerDist;
    }

    /**
     * Updates first frame localization estimates
     */
    public void setup() {
        resetOrientation();

        if (server != null) {
            server.subscribe("pose", (NetworkingCall<NetworkingPose>)(NetworkingPose pose) -> {
                // SmartDashboard.putNumber("poseX", pose.position.getX());
                globalPosition = pose.position.toTranslation2d();
                // globalOrientationFromTags = pose.rotation;
            });
    
            server.subscribe("note",  (NetworkingCall<NetworkingPose>)(NetworkingPose note) -> {
                // noteDistance = `
                noteRotation = note.rotation;
            });
        }
    }

    /**
     * Updates the state of the localization estimates
     * 
     * @param dt The amount of time past since the last update, in seconds
     */
    public synchronized void step(double dt) {
        globalHeading = pigeon.getRotation2d().minus(pigeonOffset);
        globalOrientation = globalHeading.minus(new Rotation2d(Math.PI / 2));

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
        }

        // Integrating robot position using swerve pose
        ChassisSpeeds swerveSpeeds = drive.getVelocities();
        Translation2d swerveVel = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        Translation2d navXVel = new Translation2d(navxGyro.getVelocityX(), navxGyro.getVelocityZ());
        Translation2d odometryVel = swerveVel.plus(navXVel).times(0.5).rotateBy(globalHeading.times(-1));
        globalPosition = globalPosition.plus(odometryVel.times(0.02));

        
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
