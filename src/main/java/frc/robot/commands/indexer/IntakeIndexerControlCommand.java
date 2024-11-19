package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class IntakeIndexerControlCommand extends Command {
    private static final double NOTE_INTAKE_DURATION = 6
    ;
    private double timer = NOTE_INTAKE_DURATION;
    private boolean hasNote = false;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private XboxController xbox;

	public IntakeIndexerControlCommand(IntakeSubsystem intake, IndexerSubsystem indexer, XboxController xbox) {
        this.intake = intake;
        this.indexer = indexer;
        this.xbox = xbox;

		addRequirements(intake, indexer);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
        SmartDashboard.putNumber("Note Timer", timer);

        if (xbox.getAButton() && timer > 0) { 
            intake.setWheelSpeed(0.60);
            indexer.intake();
        } else if (xbox.getYButton()) {
            intake.setWheelSpeed(-0.60);
            // indexer.reverse();
        } else if (xbox.getBButton()) {
            indexer.reverse();
        } else if (xbox.getRightBumper()) {
            indexer.intake();
        } else {
            intake.stop();
            indexer.stop();
        }

        if (indexer.hasNote() && timer > 0) {
            timer--;
        } else if (timer <= 0 && !indexer.hasNote()) {
            timer = NOTE_INTAKE_DURATION;
        }   
	}

	@Override
	public void end(boolean interrupted) {
        indexer.stop();
        intake.stop();
	}

	@Override
	public boolean isFinished() {
        return false;
    }
}
