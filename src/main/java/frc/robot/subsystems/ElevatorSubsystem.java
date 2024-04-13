package frc.robot.subsystems;

import static frc.robot.utility.Constants.MAX_VOLTAGE;
import static frc.robot.utility.Constants.Unit.FALCON_CPR;
import static frc.robot.utility.Constants.Unit.FALCON_TICKS;
import static frc.robot.utility.Constants.Unit.IN;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;
import frc.robot.utility.Constants.Unit;

/***
 * Uses stalling mechanism to move
 */
public class ElevatorSubsystem extends SubsystemBase {
	private static final double WINCH_SPEED = -0.5;
    //Units of rotations
	public static final double TOP_ENCODER_VALUE = 183500 * FALCON_TICKS;
	public static final double BOT_ENCODER_VALUE = -500 * FALCON_TICKS;

	public final TalonFX leftSpoolMotor = new TalonFX(18, "CANivore");
	public final TalonFX rightSpoolMotor = new TalonFX(14, "CANivore");

    private final Follower rightMotorControl = new Follower(leftSpoolMotor.getDeviceID(), true);

    private final StaticBrake brakeControl = new StaticBrake();
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    private final StatusSignal<Double> elevatorPosition = leftSpoolMotor.getPosition();
    private final StatusSignal<Double> motorCurrent = leftSpoolMotor.getSupplyCurrent();

    private final HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs()
    .withForwardLimitSource(ForwardLimitSourceValue.RemoteTalonFX)
    .withForwardLimitRemoteSensorID(leftSpoolMotor.getDeviceID())
    .withForwardLimitEnable(true)
    .withForwardLimitAutosetPositionEnable(true)
    .withForwardLimitAutosetPositionValue(10)
    .withReverseLimitSource(ReverseLimitSourceValue.RemoteTalonFX)
    .withReverseLimitRemoteSensorID(leftSpoolMotor.getDeviceID())
    .withReverseLimitEnable(true)
    .withReverseLimitAutosetPositionEnable(true)
    .withReverseLimitAutosetPositionValue(0);


    private final Slot0Configs talon = new Slot0Configs()
    .withKV(1.5)
    .withKG(0);
    // public static Slot0Configs getSpoolTalonConfig() {
    //     // Add configs here: 
    //     talon.slot0.kP = 0.05;
    //     talon.slot0.kI = 0.0;
    //     talon.slot0.kD = 0.00022;
    //     talon.slot0.kF = 0;
    //     talon.slot0.integralZone = 75;
    //     talon.slot0.allowableClosedloopError = 5;
    //     talon.slot0.maxIntegralAccumulator = 5120;

    //     talon.supplyCurrLimit.currentLimit = 40;//25
    //     talon.supplyCurrLimit.enable = true;
    //     talon.supplyCurrLimit.triggerThresholdCurrent = 50;
    //     talon.supplyCurrLimit.triggerThresholdTime = 0.1;

    //     return talon;
	// }


    public ElevatorSubsystem(){
        config();
    }

    public void config() {
        rightSpoolMotor.setControl(rightMotorControl);
        leftSpoolMotor.setInverted(true);

        leftSpoolMotor.setNeutralMode(NeutralModeValue.Brake);
        rightSpoolMotor.setNeutralMode(NeutralModeValue.Brake);

        // leftSpoolMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        // leftSpoolMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        // leftSpoolMotor.setSelectedSensorPosition(0);
        leftSpoolMotor.getConfigurator().setPosition(0.05);
        leftSpoolMotor.getConfigurator().apply(limitSwitchConfigs);
    }

    public double getPosition(){
        return elevatorPosition.getValue();
    }

	/**
	 * Sets target height of elevator
	 * @param height value from 0.0 to 1.0 (zero is bottom)
	 */
	public void setHeight(double height) {
		height = Math.min(height, 1);
        height = Math.max(0, height);

        height *= TOP_ENCODER_VALUE;
        leftSpoolMotor.setControl(positionControl.withPosition(height));
	}

	/**
	 * Raises elevator to the top
	 */
	public void raise() {
		setHeight(1);
	}

	/**
	 * Lowers elevator to the bottom
	 */
	public void lower() {
		setHeight(0);
	}

	/**
	 * Holds elevator using brake mode
	 */
	public void hold() {
		leftSpoolMotor.setControl(brakeControl);
	}

    public void elevate(double speed) {
        speed = Constants.boundPercentOutput(speed);
        leftSpoolMotor.setVoltage(speed);
    }

    public void winch() {
        elevate(WINCH_SPEED);
    }

	// /**
	//  * Checks if elevator is stalling at current limit
	//  * @param current limit in amps
	//  * @return true if stalling
	//  */
	// public boolean isStalling(double current) {
    //     return true;
	// 	// return Math.abs(motorCurrent.getValue()) > current && Math.abs(leftSpoolMotor.getSelectedSensorVelocity()) < VELOCITY_STALL_LIMIT;
	// }

	// /**
	//  * Checks if elevator is stalling at default current limit
	//  * @return true if stalling
	//  */
	// public boolean isStalling() {
	// 	return isStalling(STALL_CURRENT_LIMIT);
	// }

	// /**
	//  * Checks if elevator hasn't stalled for too long
	//  * @return true if elevator is safe to run
	//  */
	// public boolean canRun() {
	// 	return !isStalling() && stallTimer <= 0;
	// }

	// /**
	//  * Gets spool motor encoder position
	//  * @return position in Falcon integrated encoder ticks
	//  */
	// public double getEncoder() {
	// 	return elevatorPosition.getValue();
	// }

	// public void setMotors(double percent) {
	// 	unlocked = true;
	// 	leftSpoolMotor.setVoltage(percent * MAX_VOLTAGE);
	// }

	// public void unlock() {	
	// 	unlocked = true;
	// }

	// public void lock() {
	// 	unlocked = false;
	// }
	
	// @Override
	// public void periodic() {
	// 	if (!unlocked) {
	// 		if (isStalling()) {
	// 			hold();
	// 			stallTimer += 10;
	// 		} else if (stallTimer > 0) {
	// 			stallTimer--;
	// 		}
	// 	}
	// }
}
