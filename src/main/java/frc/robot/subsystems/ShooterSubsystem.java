package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.FALCON_TICKS;
import static frc.robot.utility.Constants.Unit.IN;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A linkage shooter subsystem designed to take notes from the IntakeSubsystem and shoot them into
 * the speaker
 */
@SuppressWarnings({"deprecated", "removal"})
public class ShooterSubsystem extends SubsystemBase {
	// TODO: measure these in CAD
	private static final double SHOOTER_LENGTH = 7.078305 * IN; // this is from the shooter pivot to its linkage connection
	private static final double LINKAGE_LONG_ARM_LENGTH = 6.375 * IN;
	private static final double LINKAGE_SHORT_ARM_LENGTH = 2.0 * IN;
	// The distance of the motor axis from the shooter pivot
	private static final double LINKAGE_PIVOT_DX = 5.270846*IN;
	private static final double LINKAGE_PIVOT_DY = -0.57172*IN;
	public static final double ANGLE_OFFSET = 16.9 * DEG;
	private static final double ENCODER_ABSOLUTE_ZERO = 0;
	private static final double INTAKE_SPEED = 0.6;
	private static final double DEADZONE = 0.1 * DEG;
	
	public final PIDController rotationPID = new PIDController(7500, 0, 5);

	public final WPI_TalonFX linkageMotor = new WPI_TalonFX(12, "CANivore");
	private final WPI_TalonFX topShooterMotor = new WPI_TalonFX(13, "CANivore");
	private final WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(5, "CANivore");
	private final WPI_TalonFX indexerMotor = new WPI_TalonFX(16, "CANivore");
	private final DutyCycleEncoder linkageEncoder = new DutyCycleEncoder(9);
	public static final double MIN_ANGLE = 15*DEG;
	public static final double MAX_ANGLE = 65*DEG;


	public ShooterSubsystem() {
		topShooterMotor.setNeutralMode(NeutralMode.Brake);
		bottomShooterMotor.setNeutralMode(NeutralMode.Brake);
		linkageMotor.setNeutralMode(NeutralMode.Brake);
		indexerMotor.setNeutralMode(NeutralMode.Brake);
		topShooterMotor.setInverted(true);
		bottomShooterMotor.follow(topShooterMotor);
		bottomShooterMotor.setInverted(true);
		linkageEncoder.reset();
	}

	/**
	 * Sets percent output for linkage motor
	 * @param percent
	 */
	public void setLinkageMotor(double percent) {
		linkageMotor.set(ControlMode.PercentOutput, percent);
	}

	/**
	 * Gets shooter rotation angle
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return linkageEncoder.getAbsolutePosition() * 2.0 * Math.PI + -2.55;
	}

	private double lawOfCosines(double a, double b, double c) {
		return Math.acos((Math.pow(c, 2.0) - Math.pow(a, 2.0) - Math.pow(b, 2.0)) / (-2.0 * a * b));
	}
	/**
	 * Sets target angle of the shooter
	 * 
	 * @param set_angle in radians (zero is parallel to the ground, positive is up)
	 */
	public void setRotation(double angle) {
		double deadzone = 0.015; // maybe decrease
		// SmartDashboard.putNumber("counter2", SmartDashboard.getNumber("counter2", 0.0) + 1);
		// SmartDashboard.putNumber("angle", angle);
		if(angle < MIN_ANGLE || angle > MAX_ANGLE) {
			return;
		}
		// trust me, the math is right
		// angle += ANGLE_OFFSET;
		double shooter_x = SHOOTER_LENGTH * Math.cos(angle);
		double shooter_y = SHOOTER_LENGTH * Math.sin(angle);
		double dx = shooter_x - LINKAGE_PIVOT_DX;
		double dy = shooter_y - LINKAGE_PIVOT_DY;
		double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		double theta1 = lawOfCosines(d, LINKAGE_SHORT_ARM_LENGTH, LINKAGE_LONG_ARM_LENGTH);
		double theta2 = Math.atan2(dx, dy);
		double angleFromY = theta1 - theta2;
		double targetRotation = Math.PI / 2 - angleFromY;
		targetRotation %= 2 * Math.PI;

        if (targetRotation < -Math.PI / 2) {
             targetRotation += 2 * Math.PI;
        } else if (targetRotation > 3 * Math.PI / 2) {
            targetRotation -= 2 * Math.PI;
        }
		double in_rotation = lawOfCosines(LINKAGE_LONG_ARM_LENGTH, LINKAGE_SHORT_ARM_LENGTH, d);
		if(targetRotation - deadzone < getRotation() && getRotation() < targetRotation + deadzone){
			linkageMotor.set(0.061 * Math.sin(in_rotation));
			// SmartDashboard.putBoolean("Dead", true);
			return;
		}
		// SmartDashboard.putBoolean("Dead", false);
		
		double linkageMag = rotationPID.calculate(getRotation(), targetRotation);
        double PID = Math.copySign(Math.min(Math.abs(linkageMag), 40 / FALCON_TICKS), linkageMag);
		double FF = ((PID < 0) ? -0.04 : 0.04);
		linkageMotor.set(ControlMode.Velocity, PID,
			DemandType.ArbitraryFeedForward, FF
		);
		SmartDashboard.putNumber("Target Shooter Angle", targetRotation / DEG);
		SmartDashboard.putNumber("Shooter PID", PID);
		SmartDashboard.putNumber("Shooter FF", FF);
	}
	

	/**
	 * Shoots at specified motor percentage
	 * @param percent
	 */
	public void shoot(double percent) {
		topShooterMotor.set(ControlMode.PercentOutput, percent);
	}

	/**
	 * Holds angle with brake mode
	 */
	public void holdAngle() {
		linkageMotor.set(ControlMode.Velocity, 0);
	}

	/**
	 * Runs shooter indexer at specified motor percentage
	 * @param percent
	 */
	public void index(double percent) {
		indexerMotor.set(ControlMode.PercentOutput, percent);
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
		indexerMotor.set(ControlMode.PercentOutput, 0);
	}

}
