package frc.robot.commands.shooter;

import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RevShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double INDEX_SPEED = 0.5;
    // public static final double REV_DURATION = 3;
    public static final double SHOOT_SPEED = 0.8;
    public ShooterSubsystem shooter;
    public double speed;
    // public double endTime;
    
    public RevShooterCommand(double speed, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // endTime = Timer.getFPGATimestamp() + REV_DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(SHOOT_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.shootCoastToggle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.getShooterVelocity() >= speed;
    }
}