package frc.robot.commands.auto;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer;
    private int SHOOT_TICKS = 10;
    private int cur_ticks = 0;
    // public double endTime;
    
    public ShootCommand(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;
        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.intake();
        indexer.intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cur_ticks++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.shootCoastToggle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cur_ticks >= SHOOT_TICKS;
    }
}