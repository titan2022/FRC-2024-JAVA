package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.FALCON_TICKS;
import static frc.robot.utility.Constants.Unit.IN;
import static frc.robot.utility.Constants.Unit.ROT;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * A linkage shooter subsystem designed to take notes from the IntakeSubsystem and shoot them into
 * the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
	// TODO: measure these in CAD
	private static final double SHOOTER_LENGTH = 7.078305 * IN; // this is from the shooter pivot to its linkage connection
	private static final double LINKAGE_LONG_ARM_LENGTH = 6.375 * IN;
	private static final double LINKAGE_SHORT_ARM_LENGTH = 2.0 * IN;
	// The distance of the motor axis from the shooter pivot
	private static final double LINKAGE_PIVOT_DX = 5.270846*IN;
	private static final double LINKAGE_PIVOT_DY = -0.57172*IN;
	private static final double ANGLE_OFFSET = 16.9 * DEG;
	private static final double INTAKE_SPEED = 0.6;
    private static final double SHOOTER_GEAR_RATIO = 2;
    private static final double SHOOTER_WHEEL_RADIUS = 2 * IN;
    private static final double ENCODER_OFFSET = -2.55;
    private static final double DEAD_ZONE = 0.015;
    public static final double MIN_ANGLE = 15*DEG;
    public static final double MAX_ANGLE = 65*DEG;

	
	private final PIDController rotationPID = new PIDController(7500, 0, 5);
	// private DoubleLogEntry angleLog;
	// private BooleanLogEntry hasShot;
    
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(16);
	public final TalonFX linkageMotor = new TalonFX(12, "CANivore");
	private final TalonFX topShooterMotor = new TalonFX(13, "CANivore");
	private final TalonFX bottomShooterMotor = new TalonFX(5, "CANivore");
	private final TalonFX indexerMotor = new TalonFX(16, "CANivore");
	private final DutyCycleEncoder linkageEncoder = new DutyCycleEncoder(9);
    
    private final StatusSignal<Double> topShooterVelocity = topShooterMotor.getVelocity();
    private final StatusSignal<Double> botShooterVelocity = topShooterMotor.getVelocity();


	public ShooterSubsystem() {
		// this.angleLog = angleLog;
		// this.hasShot = hasShot;
		topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
		bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
		indexerMotor.setNeutralMode(NeutralModeValue.Coast);
		linkageMotor.setNeutralMode(NeutralModeValue.Brake);
		topShooterMotor.setInverted(true);
		// bottomShooterMotor.follow(topShooterMotor);
		bottomShooterMotor.setInverted(true);
		linkageEncoder.reset();
	}

	/**
	 * Sets percent output for linkage motor
	 * @param percent
	 */
	public void setLinkageMotor(double percent) {
        percent = Constants.boundPercentOutput(percent);
		linkageMotor.setVoltage(percent);
	}

	/**
	 * Gets shooter rotation angle
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return linkageEncoder.getAbsolutePosition() * ROT + ENCODER_OFFSET;
	}

    public double calculateShooterRotation() {
        return 0;
    }

	private double lawOfCosines(double a, double b, double c) {
		return Math.acos((Math.pow(c, 2.0) - Math.pow(a, 2.0) - Math.pow(b, 2.0)) / (-2.0 * a * b));
	}
	/**
	 * Sets target angle of the shooter
	 * 
	 * @param set_angle in radians (zero is parallel to the ground, positive is up)
	 */
	public boolean setRotation(double angle) {
		// SmartDashboard.putNumber("counter2", SmartDashboard.getNumber("counter2", 0.0) + 1);
		// SmartDashboard.putNumber("angle", angle);
		if(angle < MIN_ANGLE || angle > MAX_ANGLE) {
			return false;
		}
		// trust me, the math is right
		angle += ANGLE_OFFSET;
		double shooter_x = SHOOTER_LENGTH * Math.cos(angle);
		double shooter_y = SHOOTER_LENGTH * Math.sin(angle);
		double dx = LINKAGE_PIVOT_DX - shooter_x;
		double dy = shooter_y - LINKAGE_PIVOT_DY;
		double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		double theta1 = lawOfCosines(d, LINKAGE_SHORT_ARM_LENGTH, LINKAGE_LONG_ARM_LENGTH);
		double theta2 = Math.atan2(dx, dy);
		double angleFromY = theta1 - theta2;
		double targetRotation = Math.PI - angleFromY;
		targetRotation %= 2 * Math.PI;

        if (targetRotation < -Math.PI / 2) {
             targetRotation += 2 * Math.PI;
        } else if (targetRotation > 3 * Math.PI / 2) {
            targetRotation -= 2 * Math.PI;
        }
		double in_rotation = lawOfCosines(LINKAGE_LONG_ARM_LENGTH, LINKAGE_SHORT_ARM_LENGTH, d);
		if(targetRotation - DEAD_ZONE < getRotation() && getRotation() < targetRotation + DEAD_ZONE){
			// linkageMotor.set(0.061 * Math.sin(in_rotation));
			holdAngle(angle, targetRotation, d);
			// SmartDashboard.putBoolean("Dead", true);
			return true;
		}
		// SmartDashboard.putBoolean("Dead", false);
		
		double linkageMag = rotationPID.calculate(getRotation(), targetRotation);
        double PID = Math.copySign(Math.min(Math.abs(linkageMag), 40 / FALCON_TICKS), linkageMag);
		double FF = ((PID < 0) ? -0.04 : 0.04);
		setLinkageMotor(FF);
		// angleLog.append(targetRotation / DEG);
		SmartDashboard.putNumber("Target Shooter Angle", angle / DEG);
		// SmartDashboard.putNumber("Shooter PID", PID);
		// SmartDashboard.putNumber("Shooter FF", FF);

		return false;
	}

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }
	
    public void setTargetAngle(Rotation2d setAngle) {
        targetAngle = setAngle;
    }

	/**
	 * Shoots at specified motor percentage
	 * @param percent
	 */
	public void shoot(double percent) {
        percent = Constants.boundPercentOutput(percent);
		topShooterMotor.setVoltage(percent);
        bottomShooterMotor.setVoltage(percent);
	}

    public void setShooterCoast() {
        topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setShooterBrake() {
        topShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double getShooterVelocity() {
        double averageMotorSpeed = Math.abs(topShooterVelocity.getValueAsDouble()) + Math.abs(topShooterVelocity.getValueAsDouble());
        double rotationsPerSec = (averageMotorSpeed * 10) * FALCON_TICKS;
        double metersPerSec = rotationsPerSec * SHOOTER_WHEEL_RADIUS / SHOOTER_GEAR_RATIO;
        return metersPerSec;
    }
    // public void shoot(double speed) {
    //     //Converts from rotations / sec to ticks / 100ms
    //     double motorSpeed = (speed / 10) / FALCON_TICKS;

	// 	topShooterMotor.set(ControlMode.Velocity, motorSpeed);
	// }

	/**
	 * Holds angle with brake mode
	 */
	public void holdAngle(double shooter_angle, double target_angle, double d) {
		// double x = LINKAGE_PIVOT_DX + LINKAGE_SHORT_ARM_LENGTH*Math.cos(target_angle - Math.PI / 2);
		// double y = LINKAGE_PIVOT_DY + LINKAGE_SHORT_ARM_LENGTH*Math.sin(target_angle - Math.PI / 2);
		// double d2 = new Translation2d(x, y).getNorm();
		// double theta2 = lawOfCosines(LINKAGE_LONG_ARM_LENGTH, SHOOTER_LENGTH, d2);

		// double theta3 = lawOfCosines(LINKAGE_LONG_ARM_LENGTH, LINKAGE_SHORT_ARM_LENGTH, d);
		// linkageMotor.set(0.0775 * Math.cos(shooter_angle) / Math.sin(theta2) * Math.sin(theta3));
		// SmartDashboard.putNumber("counter", SmartDashboard.getNumber("counter", 0) + 1);

        // guys velocity = 0 doesn't work
		// angle shifts down by a couple degrees, pls trust me on this
		linkageMotor.setVoltage(0);
	}

	/**
	 * Runs shooter indexer at specified motor percentage
	 * @param percent
	 */
	public void index(double percent) {
        percent = Constants.boundPercentOutput(percent);
		indexerMotor.setVoltage(percent);
	}

	/**
	 * Intakes note into the shooter with indexer
	 */
	public void intake() {
		index(INTAKE_SPEED);
	}

	/**
	 * Removes note from the shooter with indexer
	 */
	public void reverseIndex() {
		index(-INTAKE_SPEED);
	}

	/**
	 * Holds note in place with break mode
	 */
	public void holdIndex() {
		indexerMotor.setVoltage(0);
	}

    @Override 
    public void periodic() {
        setRotation(targetAngle.getRadians());
    }

}