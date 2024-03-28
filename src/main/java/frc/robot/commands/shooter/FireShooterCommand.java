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
public class FireShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public static final double DELAY_TIME = 0.25;
    public static final double FIRE_TIME = 1.5;
    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer;
    // public double delayTime;
    public double endTime;
    
    public FireShooterCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.indexer = indexer;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // delayTime = Timer.getFPGATimestamp() + DELAY_TIME;
        endTime = Timer.getFPGATimestamp() + FIRE_TIME;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(0.5);
        shooter.intake();
        indexer.intake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.holdIndex();
        indexer.stop();
        shooter.shoot(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }
}