package frc.robot.commands.auto;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class AutoIntakeCommand extends Command {
    public static final int INTAKE_TICKS = 6;
    private int cur_ticks = INTAKE_TICKS;
    private IndexerSubsystem indexer;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    public boolean isTriggered = false;
    public double endTime;

    public AutoIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(cur_ticks > 0){
            intake.intake();
            indexer.auto_intake_yes = true;
        } else {
            intake.stop();
            indexer.auto_intake_yes = false;
        }
        if (indexer.hasNote() && cur_ticks > 0) {
            cur_ticks--;
        } else if (cur_ticks <= 0 && !indexer.hasNote()) {
            cur_ticks = INTAKE_TICKS;
        }   

    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
