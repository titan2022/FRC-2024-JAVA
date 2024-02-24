// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_TICKS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
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

	private double linkageEncoderValue = 0;
	private double targetRotation = 0;
	
	private PIDController linkagePID = new PIDController(1, 0, 0);
	private PIDController shooterPID = new PIDController(1, 0, 0);

	private WPI_TalonFX linkageMotor = new WPI_TalonFX(0);
	private WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(0);
	private WPI_TalonFX topShooterMotor = new WPI_TalonFX(0);

	public ShooterSubsystem() {
		bottomShooterMotor.follow(topShooterMotor);
		bottomShooterMotor.setInverted(true);

		linkageMotor.setSelectedSensorPosition(0);
		topShooterMotor.setSelectedSensorPosition(0);
	}

	/**
	 * Sets linkage rotation velocity
	 * 
	 * @param velocity in radians per second
	 */
	private void setRotationVelocity(double velocity) {
		linkageMotor.set(ControlMode.Velocity, velocity / FALCON_TICKS / 10.0);
	}

	/**
	 * Gets shooter rotation angle
	 * 
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return linkageEncoderValue; // TODO
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
		double theta1 = Math.acos( // Law of Cosines, can also be negative if we want the linkage "inside"
			(Math.pow(LINKAGE_LONG_ARM_LENGTH, 2) - Math.pow(d, 2) - Math.pow(LINKAGE_SHORT_ARM_LENGTH, 2)) / 
			(-2 * d * LINKAGE_SHORT_ARM_LENGTH)
		);
		double theta2 = Math.atan2(dx, dy);
		targetRotation = Math.PI / 2 - (theta1 + theta2); // TODO
	}

	public void shoot(double velocity) {
		double currentVelocity = topShooterMotor.getSelectedSensorVelocity();
		double newVelocity = shooterPID.calculate(currentVelocity, velocity);
		topShooterMotor.set(ControlMode.Velocity, newVelocity);
	}

	@Override
	public void periodic() {
		linkageEncoderValue = linkageMotor.getSelectedSensorPosition();

		double linkageVelocity = linkagePID.calculate(getRotation(), targetRotation); // TODO
		setRotationVelocity(linkageVelocity);
	}
}
