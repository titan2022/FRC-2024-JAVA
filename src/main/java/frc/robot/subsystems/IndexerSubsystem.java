package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"deprecated", "removal"})
public class IndexerSubsystem extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.7;
	private static final int BREAK_TIMEOUT = 1; // In frames (20ms)

	private final WPI_TalonFX intakeMotor = new WPI_TalonFX(22);
	private final AnalogInput beamBreakerInput = new AnalogInput(1);

	private boolean noteStatus = false;
	private int noteStatusTimer = 0;
	private int lastNoteChange = 0;

	public IndexerSubsystem() {
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor.setInverted(true);
	}

	public void stop() {
		intakeMotor.set(ControlMode.PercentOutput, 0);
	}

	public void index(double speed) {
		intakeMotor.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward, speed);
	}

	public void intake() {
		index(INTAKE_SPEED);
	}

	public void reverse() {
		index(-INTAKE_SPEED);
	}

	public boolean hasNote() {
		return noteStatus;
	}

	@Override
	public void periodic() {
		// Beam breaker needs to be in a state for more than its timeout to count (prevents noise)
		boolean newNoteStatus = beamBreakerInput.getValue() > 2350;
		if (newNoteStatus != noteStatus && noteStatusTimer - lastNoteChange > BREAK_TIMEOUT) {
			noteStatus = newNoteStatus;
			lastNoteChange = noteStatusTimer;
		}
		noteStatusTimer++;
	}
}
