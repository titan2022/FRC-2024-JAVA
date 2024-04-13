package frc.robot.subsystems;

import static frc.robot.utility.Constants.MAX_VOLTAGE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class IndexerSubsystem extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.7;
    private static final double OUTAKE_SPEED = -1;
    private static final double NOTE_INTAKE_DURATION = 6;
	// private static final int BREAK_TIMEOUT = 1; // In frames (20ms)
    
	private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(22);
	private final AnalogInput beamBreakerInput = new AnalogInput(0);
    
    private double timer = NOTE_INTAKE_DURATION;
	public boolean auto_intake_yes = false;
	public boolean auto_shooter_yes = false;

    //Ticks or 6 * 20ms theoretically

	public IndexerSubsystem() {
		intakeMotor.setNeutralMode(NeutralMode.Brake);
		intakeMotor.setInverted(true);
	}

	public void stop() {
		intakeMotor.setVoltage(0);
	}

	public void index(double speed) {
        speed = Constants.boundPercentOutput(speed);
		intakeMotor.setVoltage(speed);
	}

	public void intake() {
		index(INTAKE_SPEED);
	}

	public void reverse() {
		index(OUTAKE_SPEED);
	}

	// public boolean hasNote() {
	// 	return noteStatus;
	// }

    // public void amp(boolean reverse) {
    //     motor.set(ControlMode.Velocity, (reverse ? -1 : 1) * AMP_VELOCITY);
    // }

    public boolean hasNote() {
        // SmartDashboard.putNumber("Beam Breaker", beamBreakerInput.getValue());
        if (beamBreakerInput.getValue() > 2345)
            return true;
        else 
            return false;
        // return false;
    }

    @Override
    public void periodic() {
        // Beam breaker needs to be in a state for more than its timeout to count (prevents noise)
        if (hasNote() && timer > 0) {
            timer--;
        } else if (!hasNote() && timer <= 0) {
            timer = NOTE_INTAKE_DURATION;
        }   
    }
}
