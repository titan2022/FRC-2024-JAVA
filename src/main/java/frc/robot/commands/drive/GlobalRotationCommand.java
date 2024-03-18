package frc.robot.commands.drive;

import static frc.robot.utility.Constants.Unit.DEG;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class GlobalRotationCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double DEAD_BAND = 2 * DEG;
    public static final Rotation2d OMEGA = Rotation2d.fromDegrees(25);
    private RotationalDrivebase drivebase;
    private Localizer localizer;
    private Rotation2d omega;
    private Rotation2d theta;
    private Rotation2d deltaAngle;
    // private double time;
    // private double endTime;

    //Theta is angle in global orientation
    public GlobalRotationCommand(Rotation2d theta, Rotation2d omega, RotationalDrivebase driveBase, Localizer localizer) {
        this.drivebase = driveBase;
        this.localizer = localizer;
        this.theta = theta;
        this.omega = omega;
        // time = theta.getRadians() / this.omega.getRadians();

        addRequirements(driveBase);
    }

    public GlobalRotationCommand(Rotation2d theta, RotationalDrivebase driveBase, Localizer localizer) {
        this(theta, OMEGA, driveBase, localizer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        deltaAngle = theta.minus(localizer.getBoundedOrientation());
        drivebase.setRotationalVelocity(new Rotation2d(Math.copySign(omega.getRadians(), deltaAngle.getRadians())));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivebase.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(deltaAngle.getRadians()) <= DEAD_BAND) 
            return true;
        else
            return false;
    }
}