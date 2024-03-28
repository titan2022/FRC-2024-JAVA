package frc.robot.commands.auto;

import frc.robot.commands.shooter.ShooterSpeakerAlgCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Localizer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ContinuousAlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer;
    public Localizer localizer;
    // public double endTime;
    
    public ContinuousAlignCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.localizer = localizer;
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
        ShooterSpeakerAlgCommand.getShootVector(localizer);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.shootCoastToggle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}