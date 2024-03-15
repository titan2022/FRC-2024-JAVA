package frc.robot.commands.shooter;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public static final double SPEAKER_HEIGHT = 2 * METERS;
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;

    public ShooterSubsystem shooter;
    public Localizer localizer;
    public boolean reachedAngle = false;
    
    public ShooterAlignSpeakerCommand(ShooterSubsystem shooter, Localizer localizer) {
        this.shooter = shooter;
        this.localizer = localizer;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double horizontalDistance = localizer.getSpeakerDistance();
        Translation2d shootVector = new Translation2d(horizontalDistance, SPEAKER_HEIGHT);
        Rotation2d shootAngle = shootVector.getAngle();

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