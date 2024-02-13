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
	 * Sets target angle of the intake
	 * 
	 * @param angle in radians (zero is ground, positive is up)
	 */
	public void setRotation(double angle) {
		targetRotation = angle; // TODO
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
