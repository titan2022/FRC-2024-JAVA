package frc.robot.commands.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class NoteIntakeCommand extends Command {
    public static final double INTAKE_DURATION = 2;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private boolean isReverse = false;
    public double endTime;

    public NoteIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter,boolean isReverse) {
        this.intake = intake;
        this.indexer = indexer;
        this.isReverse = isReverse;
        this.shooter = shooter;

        addRequirements(intake, indexer);
    }

    public NoteIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {
        this(indexer, intake, shooter, false);
    }

    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + INTAKE_DURATION;
    }

    @Override
    public void execute() {
        if (!isReverse) {
            intake.intake();
            indexer.intake();
            shooter.reverseIndex();
        } else {
            intake.reverse();
            indexer.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
        shooter.holdIndex();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
        // return indexer.hasNote();
    }
}
