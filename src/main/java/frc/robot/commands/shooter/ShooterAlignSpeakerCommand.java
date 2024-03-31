package frc.robot.commands.shooter;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.IN;
import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterAlignSpeakerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private static final double GRAVITY_STRENGTH = 9.8 * METERS;
    private static final double SPEAKER_HEIGHT = 2.0 * METERS;
    private static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.95 * IN, 7.89 * IN); 
    private static final double SHOOTER_ARM_LENGTH = 8.387 * IN;
    // public static final Translation2d TARGET_OFFSET = new Translation2d(0.92, 0.42);
    public static final Translation2d TARGET_OFFSET = new Translation2d(0.15, 0.15);



    private ShooterSubsystem shooter;
    private Localizer localizer;
    private double speed;
    private boolean reachedAngle = false;
    private Rotation2d targetAngle;
    private Rotation2d lastAngle = Rotation2d.fromDegrees(30);
    

    public static Rotation2d calculateAngle(double speed, Translation2d shootTarget) {
        double x = shootTarget.getX();
        double y = shootTarget.getY();
        double gx2OverV = GRAVITY_STRENGTH * x * x / (speed * speed);
        //Quadratic formula 
        double middleTerm = (GRAVITY_STRENGTH * GRAVITY_STRENGTH * Math.pow(x, 4) / (Math.pow(speed, 4)));
        double lastTerm = 2 * GRAVITY_STRENGTH * x * x * y / (speed * speed);
        double sqrt = Math.sqrt(x * x - middleTerm - lastTerm);
        double numerator = x - sqrt;

        return new Rotation2d(Math.atan2(numerator, gx2OverV));
    }

    public static Rotation2d linearShooter(Translation2d shootTarget) {
        return new Rotation2d(Math.atan2(shootTarget.getX(), shootTarget.getY()));
    }


    public static Translation2d getShootVector(Localizer localizer) {
        double horizontalDistance = localizer.getSpeakerPosition().getNorm();
        // double horizontalDistance = SmartDashboard.getNumber("Speaker Distance", 0) * IN;

        Translation2d robotDistanceToAprilTag = new Translation2d(horizontalDistance, SPEAKER_HEIGHT);
        Translation2d shootVector = robotDistanceToAprilTag.minus(SHOOTER_PIVOT_OFFSET);
        Rotation2d dynamicStartAngle = linearShooter(robotDistanceToAprilTag);
        // Translation2d dynamicShooterStartPoint = new Translation2d(dynamicStartAngle.getCos() * SHOOTER_ARM_LENGTH, dynamicStartAngle.getSin() * SHOOTER_ARM_LENGTH);
        Translation2d dynamicShooterStartPoint = new Translation2d(0, 0);

        Translation2d trueShootVector = shootVector.minus(dynamicShooterStartPoint);
        return trueShootVector.plus(TARGET_OFFSET);
    }
    
    public ShooterAlignSpeakerCommand(double speed, ShooterSubsystem shooter, Localizer localizer) {
        this.shooter = shooter;
        this.localizer = localizer;
        this.speed = speed;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Translation2d shootTarget = getShootVector(localizer);
        // // Translation2d shootTarget = new Translation2d(80 * IN, SPEAKER_HEIGHT);
        // SmartDashboard.putNumber("Shoot X", shootTarget.getX());
        // SmartDashboard.putNumber("Shoot Y", shootTarget.getY());

        // try {
        //     targetAngle = calculateAngle(speed, shootTarget);
        // } catch (Exception e) {
        //     targetAngle = Rotation2d.fromDegrees(30);
        // }        

        // SmartDashboard.putNumber("Shoot Angle", targetAngle.getDegrees());


    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (localizer.isSpeakerTagVisible()) {

            Translation2d shootTarget = getShootVector(localizer);
            // Translation2d shootTarget = new Translation2d(80 * IN, SPEAKER_HEIGHT);
            SmartDashboard.putNumber("Shoot X", shootTarget.getX());
            SmartDashboard.putNumber("Shoot Y", shootTarget.getY());
            targetAngle = calculateAngle(speed, shootTarget);
            SmartDashboard.putNumber("Shoot Angle", targetAngle.getDegrees());
            // reachedAngle = shooter.setRotation(targetAngle.getRadians());     
            shooter.setTargetAngle(targetAngle);
            lastAngle = targetAngle;
        } else {
            shooter.setTargetAngle(lastAngle);
        }
        // shooter.setTargetAngle(new Rotation2d(60*DEG));
        // reachedAngle = shooter.setRotation(targetAngle.getRadians());     

    }

    @Override
    public void end(boolean interrupted) {
        // shooter.holdAngle(SHOOTER_ARM_LENGTH, METERS, speed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return !localizer.isSpeakerTagVisible();
        return false;
    }
}