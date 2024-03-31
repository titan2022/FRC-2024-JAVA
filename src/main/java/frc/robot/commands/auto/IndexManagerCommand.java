package frc.robot.commands.auto;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IndexManagerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public IndexerSubsystem indexer;
    // public double endTime;
    
    public IndexManagerCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        // addRequirements(shooter);
        addRequirements(indexer);
    }
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(indexer.auto_intake_yes || indexer.auto_shooter_yes) indexer.intake();
        else indexer.stop();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        // shooter.shoot(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}