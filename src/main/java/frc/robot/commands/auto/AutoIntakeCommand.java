package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class AutoIntakeCommand extends Command {
    public static final int INTAKE_TICKS = 5;
    private int cur_ticks = 0;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    public boolean isTriggered = false;
    public double endTime;

    public AutoIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.indexer = indexer;
        // this.isReverse = isReverse;
        this.shooter = shooter;

        // addRequirements(intake, indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(cur_ticks <= INTAKE_TICKS){
            intake.intake();
            indexer.intake();
        } else {
            intake.stop();
            indexer.stop();
        }
        if(indexer.hasNote()) cur_ticks++;
        else cur_ticks--;

    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
        shooter.holdIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
