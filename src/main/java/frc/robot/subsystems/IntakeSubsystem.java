package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"deprecated", "removal"})
public class IntakeSubsystem extends SubsystemBase {
	private static final double INTAKE_SPEED = 0.45;
	
	private static final WPI_TalonFX wheelMotor = new WPI_TalonFX(19);

    // public static TalonFXConfiguration getIntakeTalonConfig() {
    //     TalonFXConfiguration talon = new TalonFXConfiguration();

    //     talon.supplyCurrLimit.currentLimit = 25;
    //     talon.supplyCurrLimit.enable = true;
    //     talon.supplyCurrLimit.triggerThresholdCurrent = 30;
    //     talon.supplyCurrLimit.triggerThresholdTime = 0.0;

    //     return talon;
	// }

	public IntakeSubsystem() {
		wheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		wheelMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		wheelMotor.setInverted(false);
		wheelMotor.setNeutralMode(NeutralMode.Brake);
	}

	/***
	 * Sets the speed of the wheels
	 * @param speed in Falcon ticks per 100ms
	 */
	public void setWheelSpeed(double speed) {
		wheelMotor.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward, speed);
	}

	public double getWheelSpeed() {
		return wheelMotor.getSelectedSensorVelocity();
	}

	public void toggle() {
		if (Math.abs(getWheelSpeed()) <= 0.01) {
			intake();
		} else {
			stop();
		}
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
