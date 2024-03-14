package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class RotationCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final Rotation2d deadBand = Rotation2d.fromDegrees(1);
    public static final Rotation2d OMEGA = Rotation2d.fromDegrees(45);
    private RotationalDrivebase drivebase;
    private Localizer localizer;
    private Rotation2d omega;
    private Rotation2d theta;
    private Rotation2d targetAngle;
    // private double time;
    // private double endTime;

    public RotationCommand(Rotation2d theta, Rotation2d omega, RotationalDrivebase driveBase, Localizer localizer) {
        this.drivebase = driveBase;
        this.localizer = localizer;
        this.theta = theta;
        this.omega = new Rotation2d(Math.copySign(omega.getRadians(), theta.getRadians()));
        // time = theta.getRadians() / this.omega.getRadians();

        addRequirements(driveBase);
    }

    public RotationCommand(Rotation2d theta, RotationalDrivebase driveBase, Localizer localizer) {
        this(theta, OMEGA, driveBase, localizer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetAngle = localizer.getHeading().plus(theta);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivebase.setRotationalVelocity(omega);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (theta.getRadians() > 0 && targetAngle.minus(localizer.getHeading()).getRadians() <= deadBand.getRadians())
            return true;
        else if (theta.getRadians() <= 0 && targetAngle.minus(localizer.getHeading()).getRadians() >= deadBand.getRadians())
            return true;
        else 
            return false;
    }
}