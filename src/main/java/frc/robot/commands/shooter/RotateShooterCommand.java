package frc.robot.commands.shooter;

import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RotateShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public ShooterSubsystem shooter;
    public Rotation2d angle;
    public boolean reachedAngle = false;
    
    public RotateShooterCommand(Rotation2d angle, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("2", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // shooter.setTargetAngle(angle);
        reachedAngle = shooter.setRotation(angle.getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putBoolean("2end", true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedAngle;
    }
}