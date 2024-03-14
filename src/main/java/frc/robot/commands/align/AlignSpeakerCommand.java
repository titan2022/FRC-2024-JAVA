package frc.robot.commands.align;

import static frc.robot.utility.Constants.Unit.DEG;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.Constants;
import frc.robot.commands.drive.RotationCommand;
import frc.robot.commands.drive.TranslationCommand;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class AlignSpeakerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;

    // public static final double TRANSLATIONAL_SPEED = 1;
    public static final double ROTATIONAL_SPEED = 25 * DEG;
    public static final double ANGLE_DEADBAND = 5 * DEG;
    public static final double SUBWOOFER_LENGTH = 0.91;
    public RotationalDrivebase rotational;
    public Localizer localizer;
    public Rotation2d aprilTagAngle;

    public AlignSpeakerCommand(RotationalDrivebase rotationalDrive, Localizer localizer) {
        this.rotational = rotationalDrive;
        this.localizer = localizer;
        addRequirements(rotationalDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Constants.getColor() == Alliance.Blue) {
            // distanceFromAprilTag = localizer.getTagPosition(BLUE_SPEAKER_APRILTAG);
            aprilTagAngle = localizer.getTagHeading(BLUE_SPEAKER_APRILTAG);
        } else {
            // distanceFromAprilTag = localizer.getTagPosition(RED_SPEAKER_APRILTAG);
            aprilTagAngle = localizer.getTagHeading(RED_SPEAKER_APRILTAG);
        }
        Rotation2d speed = new Rotation2d(Math.copySign(ROTATIONAL_SPEED, aprilTagAngle.getRadians()));
        SmartDashboard.putNumber("Align Omega", speed.getDegrees());
        rotational.setRotationalVelocity(speed);
        SmartDashboard.putBoolean("Align End", false);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Align End", true);
        rotational.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(aprilTagAngle.getRadians()) < ANGLE_DEADBAND) 
            return true;
        else 
            return false;
    }
}

