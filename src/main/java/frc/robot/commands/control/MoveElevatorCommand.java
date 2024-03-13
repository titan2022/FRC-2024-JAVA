package frc.robot.commands.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
public class MoveElevatorCommand extends Command {
	private final static double HIGH_SPEED = 0.4;
	private final static double LOW_SPEED = 0.2;
	private final static double HIGH_SPEED_TIME = 1;

	private ElevatorSubsystem elevator;
	private double highSpeedTime;
	private boolean up;

	public MoveElevatorCommand(boolean up, ElevatorSubsystem elevator) {
		this.elevator = elevator;
		this.up = up;

		addRequirements(elevator);
	}

	@Override
	public void initialize() {
		highSpeedTime = Timer.getFPGATimestamp() + HIGH_SPEED_TIME;
	}

	@Override
	public void execute() {
		double speed = 1;
		if (!up)  {
			speed *= -1;
		}

		speed *= LOW_SPEED;

		elevator.elevate(speed);
	}

	@Override
	public void end(boolean interrupted) {
		elevator.hold();
	}

	@Override
	public boolean isFinished() {
		if (elevator.canRun()) {
			if (up && elevator.getEncoder() < ElevatorSubsystem.TOP_ENCODER_VALUE)
				return false;
			else if (!up && elevator.getEncoder() > ElevatorSubsystem.BOT_ENCODER_VALUE)
				return false;
			else
				return true;
		} else {
			return true;
		}
	}
}
