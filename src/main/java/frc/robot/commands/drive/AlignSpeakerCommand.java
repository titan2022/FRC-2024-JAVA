package frc.robot.commands.drive;

import static frc.robot.utility.Constants.Unit.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class AlignSpeakerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private static final double DEAD_BAND = 2 * DEG;
    public static final Rotation2d OMEGA = Rotation2d.fromDegrees(35);
    private static final Rotation2d SPIN_OFFSET = Rotation2d.fromDegrees(0);
    private RotationalDrivebase drivebase;
    private Localizer localizer;
    private Rotation2d omega;
    private Rotation2d deltaAngle;
    private Rotation2d setAngle;

    // private double time;
    // private double endTime;

    public AlignSpeakerCommand(RotationalDrivebase driveBase, Localizer localizer) {
        this.drivebase = driveBase;
        this.localizer = localizer;
        // time = theta.getRadians() / this.omega.getRadians();

        addRequirements(driveBase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Translation2d speakerPosition = localizer.getSpeakerPosition();
        // double angle = Math.tan(speakerPosition.getX() / speakerPosition.getY()) - (Math.PI / 2);
        // double rotationalAngle = 3 * Math.PI / 2;
        // setAngle = new Rotation2d(rotationalAngle + angle);
        // deltaAngle = new Rotation2d(setAngle.getRadians() - localizer.getTrueOrientation().getRadians());
        // angle -= Math.PI / 2;
        // Rotation2d translationalDeltaAngle = new Rotation2d(angle);
        // Rotation2d rotationalDeltaAngle = localizer.getSpeakerHeading();
        // deltaAngle = translationalDeltaAngle.minus(rotationalDeltaAngle).plus(SPIN_OFFSET);
        // Rotation2d translationalDeltaAngle = new Rotation2d(angle);
        // Rotation2d rotationalDeltaAngle = localizer.getSpeakerHeading();
        // deltaAngle = translationalDeltaAngle.minus(rotationalDeltaAngle);
        // setAngle = localizer.getOrientation().plus(deltaAngle.plus(SPIN_OFFSET));
        // setAngle = new Rotation2d(setAngle.getRadians() % (2 * Math.PI));

        // SmartDashboard.putNumber("Translational Angle", translationalDeltaAngle.getDegrees());
        // SmartDashboard.putNumber("Rotational Angle", rotationalDeltaAngle.getDegrees());
        // SmartDashboard.putNumber("Total Angle", deltaAngle.getDegrees());

        deltaAngle = localizer.getSpeakerHeading().plus(SPIN_OFFSET);
        setAngle = localizer.getTrueOrientation().plus(deltaAngle);

        omega = new Rotation2d(Math.copySign(OMEGA.getRadians(), deltaAngle.getRadians()));
        // omega = new Rotation2d(0);


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivebase.setRotationalVelocity(omega);

        SmartDashboard.putNumber("delta angle", deltaAngle.getDegrees());
        SmartDashboard.putNumber("set angle", setAngle.getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (omega.getRadians() > 0) {
            return (setAngle.getRadians() - localizer.getTrueOrientation().getRadians() <= DEAD_BAND);
        }
        else {
            return Math.abs(setAngle.getRadians() - localizer.getTrueOrientation().getRadians()) <= DEAD_BAND;
        }
    }
}
