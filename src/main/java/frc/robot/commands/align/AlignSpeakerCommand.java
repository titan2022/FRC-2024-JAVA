package frc.robot.commands.align;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.METERS;

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
    public static final double ANGLE_DEADBAND = 2.5 * DEG;
    public static final double SUBWOOFER_LENGTH = 0.91 * METERS;
    private static final double OMEGA = 30 * DEG;
    public RotationalDrivebase rotational;
    public Localizer localizer;
    private Rotation2d targetAngle;
    public Rotation2d deltaAngle;
    public Rotation2d omega;

    public AlignSpeakerCommand(RotationalDrivebase rotationalDrive, Localizer localizer) {
        this.rotational = rotationalDrive;
        this.localizer = localizer;

        addRequirements(rotationalDrive);
    }

    @Override
    public void initialize() {
        if (Constants.getColor() == Alliance.Blue) {
            targetAngle = localizer.getBlueSpeaker().toTranslation2d().getAngle();
        } else {
            targetAngle = localizer.getRedSpeaker().toTranslation2d().getAngle();  
        }
        deltaAngle = targetAngle.minus(localizer.getBoundedOrientation());
        omega = new Rotation2d(Math.copySign(OMEGA, deltaAngle.getRadians()));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        rotational.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Constants.getColor() == Alliance.Blue) {
            targetAngle = localizer.getBlueSpeaker().toTranslation2d().getAngle();
        } else {
            targetAngle = localizer.getRedSpeaker().toTranslation2d().getAngle();  
        }
        deltaAngle = targetAngle.minus(localizer.getBoundedOrientation());

        if (Math.abs(deltaAngle.getRadians()) <= ANGLE_DEADBAND) 
            return true;
        else
            return false;
    }
}

