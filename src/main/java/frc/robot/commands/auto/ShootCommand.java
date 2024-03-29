package frc.robot.commands.auto;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer;
    public IntakeSubsystem intake;
    public double FIRE_TIME = 1.0;
    public double endTime = 0;
    // public double endTime;
    
    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        // addRequirements(shooter);
        addRequirements(indexer);
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