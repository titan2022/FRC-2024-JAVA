package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_CPR;
import static frc.robot.utility.Constants.Unit.IN;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * Uses stalling mechanism to move
 */
@SuppressWarnings({"deprecated", "removal"})
public class ElevatorSubsystem extends SubsystemBase {
	private static final double STALL_CURRENT_LIMIT = 15;
	private static final double SPOOL_RADIUS = 1 * IN;
	private static final double GEAR_RATIO = 28;
	private static final double TICKS_PER_METER = FALCON_CPR * GEAR_RATIO / (10 * SPOOL_RADIUS * 2 * Math.PI);
	private static final double WINCH_SPEED = -0.5;
	public static int TOP_ENCODER_VALUE = 183500;
	public static final int BOT_ENCODER_VALUE = -500;
	private static final double VELOCITY_STALL_LIMIT = 1000;

	public final WPI_TalonFX leftSpoolMotor = new WPI_TalonFX(18, "CANivore");
	public final WPI_TalonFX rightSpoolMotor = new WPI_TalonFX(14, "CANivore");

	private int stallTimer = 0;
	public boolean unlocked = false;

    public static TalonFXConfiguration getSpoolTalonConfig() {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here: 
        talon.slot0.kP = 0.05;
        talon.slot0.kI = 0.0;
        talon.slot0.kD = 0.00022;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 75;
        talon.slot0.allowableClosedloopError = 5;
        talon.slot0.maxIntegralAccumulator = 5120;

        talon.supplyCurrLimit.currentLimit = 40;//25
        talon.supplyCurrLimit.enable = true;
        talon.supplyCurrLimit.triggerThresholdCurrent = 50;
        talon.supplyCurrLimit.triggerThresholdTime = 0.1;

        return talon;
	}


    public ElevatorSubsystem(){
        config();
    }

    public double getPosition(){
        return leftSpoolMotor.getSelectedSensorPosition();
    }

    public void config() {
        leftSpoolMotor.configAllSettings(getSpoolTalonConfig());
        rightSpoolMotor.configAllSettings(getSpoolTalonConfig());

        rightSpoolMotor.follow(leftSpoolMotor);
        leftSpoolMotor.setInverted(true);
        rightSpoolMotor.setSensorPhase(true);

        leftSpoolMotor.setNeutralMode(NeutralMode.Brake);
        rightSpoolMotor.setNeutralMode(NeutralMode.Brake);

		leftSpoolMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
		leftSpoolMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		leftSpoolMotor.setSelectedSensorPosition(0);
	}

	/**
	 * Winches elevator for climb
	 * @param currentLimit new limit specific to elevator
	 */
	public void climb(double currentLimit) {
		throw new UnsupportedOperationException("Impliment this.");
	}

	/**
	 * Winches elevator
	 */
	public void winch() {
		elevate(WINCH_SPEED);
	}

	/**
	 * Sets elevator speed
	 * @param speed in Falcon ticks per 100ms (?)
	 */
	public void elevate(double speed) {
		leftSpoolMotor.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward, speed);
	}

	/**
	 * Sets target height of elevator
	 * @param height value from 0.0 to 1.0 (zero is bottom)
	 */
	public void setHeight(double height) {
		double setPoint = Math.abs(height) * TOP_ENCODER_VALUE;
		if (setPoint > 1) {
			setPoint = 1;
		} else if (setPoint < 0) {
			setPoint = 0;
		}

		if (!unlocked) {
			if (canRun()) {
                if (setPoint == 0) {
				    leftSpoolMotor.set(ControlMode.Position,BOT_ENCODER_VALUE);
                } else {
                    leftSpoolMotor.set(ControlMode.Position, setPoint * TOP_ENCODER_VALUE);
                }
			} else {
				leftSpoolMotor.set(ControlMode.Velocity, 0);
			}
		} else {
			leftSpoolMotor.set(ControlMode.Position, setPoint * TOP_ENCODER_VALUE);
		}
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
		leftSpoolMotor.set(ControlMode.Velocity, 0);
	}

	/**
	 * Checks if elevator is stalling at current limit
	 * @param current limit in amps
	 * @return true if stalling
	 */
	public boolean isStalling(double current) {
		return Math.abs(leftSpoolMotor.getOutputCurrent()) > current && Math.abs(leftSpoolMotor.getSelectedSensorVelocity()) < VELOCITY_STALL_LIMIT;
	}

	/**
	 * Checks if elevator is stalling at default current limit
	 * @return true if stalling
	 */
	public boolean isStalling() {
		return isStalling(STALL_CURRENT_LIMIT);
	}

	/**
	 * Checks if elevator hasn't stalled for too long
	 * @return true if elevator is safe to run
	 */
	public boolean canRun() {
		return !isStalling() && stallTimer <= 0;
	}

	/**
	 * Gets spool motor encoder position
	 * @return position in Falcon integrated encoder ticks
	 */
	public double getEncoder() {
		return leftSpoolMotor.getSelectedSensorPosition();
	}

	public void setMotors(double percent) {
		unlocked = true;
		leftSpoolMotor.set(ControlMode.Velocity, percent);
	}

	public void unlock() {	
		unlocked = true;
	}

	public void lock() {
		unlocked = false;
	}
	
	@Override
	public void periodic() {
		if (!unlocked) {
			if (isStalling()) {
				hold();
				stallTimer += 10;
			} else if (stallTimer > 0) {
				stallTimer--;
			}
		}
	}
}
