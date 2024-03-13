package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class NoteIntakeCommand extends Command {
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private boolean isReverse = false;

    public NoteIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, boolean isReverse) {
        this.intake = intake;
        this.indexer = indexer;
        this.isReverse = isReverse;

        addRequirements(intake, indexer);
    }

    public NoteIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        this(indexer, intake, false);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (!isReverse) {
            intake.intake();
            indexer.intake();
        } else {
            intake.reverse();
            indexer.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return indexer.hasNote();
    }
}
