package frc.robot.commands.shooter;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.utility.Constants.Unit.IN;
import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterSpeakerAlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;
    public static final double GRAVITY_STRENGTH = 9.8 * Unit.METERS;
    public static final double SPEAKER_HEIGHT = 2.0 * Unit.METERS;
    public static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.95 * Unit.IN, 7.89 * Unit.IN); 
    public static final double SHOOTER_ARM_LENGTH = 8.387 * Unit.IN;
    public static final Translation2d TARGET_OFFSET = new Translation2d();

    public ShooterSubsystem shooter;
    public Localizer localizer;
    public double speed;
    public boolean reachedAngle = false;

    
    public static Rotation2d calculateAngle(double speed, Translation2d shootTarget) {
        double x = shootTarget.getX();
        double y = shootTarget.getY();
        double gx2OverV = GRAVITY_STRENGTH * x * x / (speed * speed);
        //Quadratic formula 
        double middleTerm = (GRAVITY_STRENGTH * GRAVITY_STRENGTH * Math.pow(x, 4) / (Math.pow(speed, 4)));
        double lastTerm = 2 * GRAVITY_STRENGTH * x * x * y / (speed * speed);
        double sqrt = Math.sqrt(x * x - middleTerm - lastTerm);

        if (Double.isNaN(sqrt))
            return Rotation2d.fromDegrees(60);
        else {
            double numerator = x - sqrt;
            return new Rotation2d(Math.atan2(numerator, gx2OverV));
        }
    }

    public static Translation2d getShootVector(Localizer localizer) {
        Translation3d speakerDisplacement;
        if (Constants.getColor() == Alliance.Blue) {
            speakerDisplacement = localizer.getBlueSpeaker();
        } else {
            speakerDisplacement = localizer.getRedSpeaker();

        }
        return speakerDisplacement.toTranslation2d().plus(TARGET_OFFSET);
    }
    
    public ShooterSpeakerAlignCommand(double speed, ShooterSubsystem shooter, Localizer localizer) {
        this.shooter = shooter;
        this.localizer = localizer;
        this.speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d shootVector = getShootVector(localizer);
        Rotation2d shootAngle = calculateAngle(speed, shootVector);

        reachedAngle = shooter.setRotation(shootAngle.getRadians());
        SmartDashboard.putBoolean("Shoot Angle Reached", reachedAngle);
        SmartDashboard.putNumber("Shoot Align Angle", shootAngle.getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedAngle;
    }
}