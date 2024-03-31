package frc.robot.utility;

import java.util.Dictionary;
import java.util.Hashtable;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.utility.localization.AprilTag;
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
    // private PhotonCamera camera = new PhotonCamera("photonvision");
    private NetworkingServer server;
    private SwerveDriveSubsystem drive;
    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(15);
    public static final AHRS navxGyro = new AHRS(SPI.Port.kMXP);


    public Rotation2d pigeonOffset = new Rotation2d();
    // public Rotation2d localOffset = new Rotation2d();

    private Translation2d globalPosition = new Translation2d();
    private Translation2d cameraBasedGlobalPosition = new Translation2d();
    private Rotation2d globalHeading = new Rotation2d();
    private Rotation2d globalOrientation = new Rotation2d();
    private Rotation2d globalOrientationFromTags = new Rotation2d();
    // private Rotation2d localOrientation = new Rotation2d();
    private double noteDistance = -1;
    private Rotation3d noteRotation = new Rotation3d();
    private Dictionary<Integer, NetworkingTag> tags = new Hashtable<>();
    private Translation2d[] speaker_location = {new Translation2d(-1.50, 218.42), new Translation2d(652.73, 218.42)};
    private boolean speakerTagVisible = false;
    private Rotation2d speakerHeading = new Rotation2d();

    private Translation2d speakerRobotPosition = new Translation2d();
    private Rotation2d speakerRobotOrientation = new Rotation2d();

    public Pose2d startingPose2d = new Pose2d(); 
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

    public Rotation2d getTrueOrientation() {
        return globalHeading.plus(new Rotation2d(Math.PI / 2));
    }

    /**
     * Resets rotational offset 
     */
    public void resetHeading() {
        // navxGyro.reset();
        // localOffset = pigeon.getRotation2d();
        resetHeading(Rotation2d.fromDegrees(0));
    }

    public void resetHeading(Rotation2d startingAngle) {
        pigeonOffset = pigeon.getRotation2d().plus(startingAngle);
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

    public void resetStartingPose2d(){
        resetStartingPose2d(new Pose2d());
    }

    public void resetPose2d(Pose2d pose){
        SmartDashboard.putNumber("reset_pose_x", pose.getX());
        globalPosition = pose.getTranslation();
        // if(DriverStation.getAlliance().equals(Alliance.Blue)){
            pigeonOffset = pigeon.getRotation2d().plus(new Rotation2d(Math.PI/2)).minus(pose.getRotation());
        // }
        // else{
        //     pigeonOffset = pigeon.getRotation2d().minus(new Rotation2d(Math.PI/2)).minus(pose.getRotation());
        // }
    }
    /**
     * 5rtf
     * Gets Pose2d based on translation and rotation for path planner
     * @return
     */
    public Pose2d getDisplacementPose2d() {
        return (new Pose2d(globalPosition,globalHeading)).relativeTo(startingPose2d);
    }

    public Pose2d getCameraBasedDisplacementPose2d() {
        return (new Pose2d(cameraBasedGlobalPosition, globalHeading)).relativeTo(startingPose2d);
    }

    // public Pose2d getAutoDisplacementPose2d(){
    //     Pose2d normalPose2d = getDisplacementPose2d();
    //     return new Pose2d(normalPose2d.getTranslation().rotateBy(new Rotation2d(Math.PI/2)), normalPose2d.getRotation());
    // }

    /**
     * Gets global rotation of speaker
     * @return Rotation2d
     */
    public Rotation2d getSpeakerHeading() {
        // return globalOrientationFromTags.times(-1.0).plus(Rotation2d.fromDegrees(16.94));
        return speakerHeading;
    }

    /**
     * Gets position from shooter to speaker in meters
     * @return meters
     */
    public Translation2d getSpeakerPosition() {
        return new Translation2d(-speakerRobotPosition.getX(), speakerRobotPosition.getY()); // Speaker is 0, 0
    }

    public Rotation2d getSpeakerOrientation(){
        return speakerRobotOrientation;
    }

    /**
     * Returns true if speaker tag is seen by camera
     * @return boolean
     */
    public boolean isSpeakerTagVisible() {
        return speakerTagVisible;
    }

    /**
     * Updates first frame localization estimates
     */
    public void setup() {
        // resetHeading();

        if (server != null) {
            server.subscribe("pose", (NetworkingCall<NetworkingPose>)(NetworkingPose pose) -> {
                SmartDashboard.putNumber("poseX", pose.position.getX());
                SmartDashboard.putNumber("poseZ", pose.position.getZ());
                speakerRobotPosition = new Translation2d(pose.position.getX(), pose.position.getZ());
                speakerRobotOrientation = Rotation2d.fromRadians(pose.rotation.getY());
            });
    
            server.subscribe("speaker",  (NetworkingCall<NetworkingPose>)(NetworkingPose speaker) -> {
                speakerHeading = Rotation2d.fromRadians(speaker.rotation.getY());
                SmartDashboard.putNumber("speakerHeading", speakerHeading.getDegrees());
            });

            server.subscribe("visible", (NetworkingCall<Translation3d>)(Translation3d fakeVec) -> {
                speakerTagVisible = fakeVec.getX() > 0;
                SmartDashboard.putBoolean("Tag Visible", speakerTagVisible);
            });
        }
    }

    private AprilTag idToTag(int id){
        AprilTag arr[] = new AprilTag[]{
            AprilTag.BLUE_SOURCE_SOUTH,
            AprilTag.BLUE_SOURCE_NORTH,
            AprilTag.RED_SOURCE_SOUTH, 
            AprilTag.RED_SOURCE_NORTH,
            AprilTag.RED_SPEAKER_SOUTH, 
            AprilTag.RED_SPEAKER_CENTER,
            AprilTag.BLUE_SPEAKER_SOUTH, 
            AprilTag.BLUE_SPEAKER_NORTH,
            AprilTag.RED_AMP, 
            AprilTag.BLUE_AMP,
            AprilTag.RED_STAGE_SOUTH, 
            AprilTag.RED_STAGE_NORTH, 
            AprilTag.RED_STAGE_WEST,
            AprilTag.BLUE_STAGE_SOUTH, 
            AprilTag.BLUE_STAGE_NORTH, 
            AprilTag.BLUE_STAGE_EAST,
        };
        for(AprilTag aprilTag : arr){
            if(aprilTag.getID() == id){
                return aprilTag;
            }
        }
        return null;
    }

    /**
     * Updates the state of the localization estimates
     * 
     * @param dt The amount of time past since the last update, in seconds
     */
    public synchronized void step(double dt) {
        globalHeading = pigeon.getRotation2d().minus(pigeonOffset);
        globalOrientation = globalHeading.minus(new Rotation2d(Math.PI / 2));
        SmartDashboard.putNumber("heading", globalHeading.getRadians());
        // var result = camera.getLatestResult();
        // if (result.hasTargets()) {
        //     PhotonTrackedTarget target = result.getBestTarget();
        //     int targetID = target.getFiducialId();
            
        //     double poseAmbiguity = target.getPoseAmbiguity();
        //     Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        //     Rotation3d toRobotRotation = new Rotation3d((90-50.5)*DEG, 1, -16.94*DEG);
        //     Translation3d translation = bestCameraToTarget.getTranslation();
        //     translation.rotateBy(toRobotRotation);
        //     Translation2d t2d = translation.toTranslation2d();
        //     t2d.rotateBy(globalHeading);
        //     t2d.plus((new Translation2d(-9.628, -11.624).rotateBy(globalHeading)));
        //     globalPosition = idToTag(targetID).getPosition().minus(t2d);
        // }

        // Integrating robot position using swerve pose
        ChassisSpeeds swerveSpeeds = drive.getVelocities();
        Translation2d swerveVel = new Translation2d(swerveSpeeds.vxMetersPerSecond, swerveSpeeds.vyMetersPerSecond);
        Translation2d navXVel = new Translation2d(0, 0);
        // there was a times .5 for some reason, lmk if that's important
        Translation2d odometryVel = swerveVel.plus(navXVel).rotateBy(globalHeading);
        SmartDashboard.putNumber("odometryvx", odometryVel.getX());
        SmartDashboard.putNumber("odometryvy", odometryVel.getY());
        globalPosition = globalPosition.plus(odometryVel.times(0.02));
    }
    
    public Translation2d getSpeakerLocation(){
        if(Constants.getColor().equals(Alliance.Blue)){
            return speaker_location[0].minus(globalPosition);
        } else {
            return speaker_location[1].minus(globalPosition);
        }
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