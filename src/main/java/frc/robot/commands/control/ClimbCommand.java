package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class ClimbCommand extends Command {
	public static final double STALL_LIMIT = 30;

	public ElevatorSubsystem elevator;

	public ClimbCommand(ElevatorSubsystem elevator) {
		this.elevator = elevator;
		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		
	}

	@Override
	public void execute() {
		elevator.climb(STALL_LIMIT);
	}

	@Override
	public void end(boolean interrupted) {
		elevator.hold();
	}

	@Override
	public boolean isFinished() {
		return elevator.isStalling(STALL_LIMIT);
	}
}
