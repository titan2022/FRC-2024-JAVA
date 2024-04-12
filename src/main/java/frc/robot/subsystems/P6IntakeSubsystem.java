package frc.robot.subsystems;

import static frc.robot.utility.Constants.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class P6IntakeSubsystem extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.6;
    
    public TalonFX wheelMotor = new TalonFX(19);
    public VoltageOut voltage = new VoltageOut(0);

    // public static TalonFXConfiguration getIntakeTalonConfig() {
    //     TalonFXConfiguration talon = new TalonFXConfiguration();

    //     talon.supplyCurrLimit.currentLimit = 25;
    //     talon.supplyCurrLimit.enable = true;
    //     talon.supplyCurrLimit.triggerThresholdCurrent = 30;
    //     talon.supplyCurrLimit.triggerThresholdTime = 0.0;

    //     return talon;
	// }

	public P6IntakeSubsystem() {
        config();
		// wheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		// wheelMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		// wheelMotor.setInverted(false);
		// wheelMotor.setNeutralMode(NeutralMode.Brake);
	}

    public void config() {
        wheelMotor.setNeutralMode(NeutralModeValue.Coast);
    }

	/***
	 * Sets the speed of the wheels
	 * @param speed in Falcon ticks per 100ms
	 */
	public void setWheelSpeed(double speed) {
        wheelMotor.setControl(voltage.withOutput(speed * MAX_VOLTAGE));
	}

	public void intake() {
		setWheelSpeed(INTAKE_SPEED);
	}

	public void reverse() {
		setWheelSpeed(-INTAKE_SPEED);
	}

	public void stop() {
		setWheelSpeed(0);
	}
}
