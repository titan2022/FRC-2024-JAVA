package frc.robot.subsystems;

import static frc.robot.utility.Constants.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class IntakeSubsystem extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.6;
    
    public TalonFX wheelMotor = new TalonFX(19);

    // public static TalonFXConfiguration getIntakeTalonConfig() {
    //     TalonFXConfiguration talon = new TalonFXConfiguration();

    //     talon.supplyCurrLimit.currentLimit = 25;
    //     talon.supplyCurrLimit.enable = true;
    //     talon.supplyCurrLimit.triggerThresholdCurrent = 30;
    //     talon.supplyCurrLimit.triggerThresholdTime = 0.0;

    //     return talon;
	// }

	public IntakeSubsystem() {
        config();
	}

    public void config() {
        wheelMotor.setNeutralMode(NeutralModeValue.Coast);
    }

	/***
	 * Sets the speed of the wheels
	 * @param speed in Falcon ticks per 100ms
	 */
	public void setWheelSpeed(double speed) {
        speed = Constants.boundPercentOutput(speed);
        wheelMotor.setVoltage(speed);
	}

	public void intake() {
		setWheelSpeed(INTAKE_SPEED);
	}

	public void reverse() {
		setWheelSpeed(-INTAKE_SPEED);
	}

	public void stop() {
		wheelMotor.setVoltage(0);
	}
}
