// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_TICKS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A linkage shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
	private static final double MAX_ANGLE = 0;
	private static final double MIN_ANGLE = 0;
	
	// TODO: measure these in CAD
	private static final double SHOOTER_LENGTH = 1.0; // this is from the shooter pivot to its linkage connection
	private static final double LINKAGE_LONG_ARM_LENGTH = 1.0;
	private static final double LINKAGE_SHORT_ARM_LENGTH = 1.0;
	// the distance of the motor axis from the shooter pivot
	private static final double LINKAGE_PIVOT_DX = 1.0;
	private static final double LINKAGE_PIVOT_DY = 1.0;

  //kP, kI, kD, kF
	public static final double[] shooterPID = {0, 0, 0, 0};
	public static final double[] rotatorPID = {0, 0, 0, 0};

	public static final double[] shooterFeedForwardParams = {0, 0, 0};
	public static final double[] rotatorFeedForwardParams = {0, 0, 0};

	SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterFeedForwardParams[0], shooterFeedForwardParams[1], shooterFeedForwardParams[2]);
	ArmFeedforward rotatorFeedforward = new ArmFeedforward(rotatorFeedForwardParams[0], rotatorFeedForwardParams[1], rotatorFeedForwardParams[2]);
		
	private WPI_TalonFX linkageMotor = new WPI_TalonFX(0);
	private WPI_TalonFX rotatorMotor = new WPI_TalonFX(0);
	private WPI_TalonFX shooterMotor = new WPI_TalonFX(0);
    private PIDController rotationPID = new PIDController(0, 0, 0);
	private DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(0);

	public ShooterSubsystem() {
    	config();
	}

  	public void config() {
    	rotatorMotor.setInverted(true);

        rotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		linkageMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		shooterMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
		rotationEncoder.reset();
  	}

	/**
	 * Gets shooter rotation angle
	 * 
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return rotationEncoder.get();
	}

	/**
	 * Sets target angle of the shooter
	 * 
	 * @param angle in radians (zero is parallel to the ground, positive is up)
	 */
	public void setRotation(double angle) {
		if(angle < MIN_ANGLE || angle > MAX_ANGLE) return;
		// trust me, the math is right
		// TODO: check my math
		double shooter_x = SHOOTER_LENGTH * Math.cos(angle);
		double shooter_y = SHOOTER_LENGTH * Math.sin(angle);
		double dx = shooter_x - LINKAGE_PIVOT_DX;
		double dy = shooter_y - LINKAGE_PIVOT_DY;
		double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
		// double theta1 = Math.acos( // Law of Cosines, can also be negative if we want the linkage "inside"
		// 	(Math.pow(LINKAGE_LONG_ARM_LENGTH, 2) - Math.pow(d, 2) - Math.pow(LINKAGE_SHORT_ARM_LENGTH, 2)) / 
		// 	(-2 * d * LINKAGE_SHORT_ARM_LENGTH)
		// );
    	double theta1 = Math.acos( // Law of Cosines, can also be negative if we want the linkage "inside"
			(-Math.pow(LINKAGE_LONG_ARM_LENGTH, 2) + Math.pow(d, 2) + Math.pow(LINKAGE_SHORT_ARM_LENGTH, 2)) / 
			(2 * d * LINKAGE_SHORT_ARM_LENGTH)
		);
		double theta2 = Math.atan2(dx, dy);
        double angleFromY = theta1 - theta2;
		double targetRotation = Math.PI / 2 - angleFromY; // TODO
        targetRotation %= 2 * Math.PI;

        if (targetRotation < -Math.PI / 2) {
             targetRotation += 2 * Math.PI;
        } else if (targetRotation > 3 * Math.PI / 2) {
            targetRotation -= 2 * Math.PI;
        }

        rotatorMotor.set(ControlMode.PercentOutput, rotationPID.calculate(getRotation(), targetRotation));
	}

	public void shoot(double velocity) {
		shooterMotor.set(ControlMode.Velocity, velocity);
	}

	public void setLinkage(double speed) {
		linkageMotor.set(ControlMode.PercentOutput, speed);
	}

	public void holdLinkage() {
		linkageMotor.set(ControlMode.PercentOutput, 0);
	}
}
