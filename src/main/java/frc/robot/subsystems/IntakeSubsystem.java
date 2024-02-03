// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.FALCON_TICKS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An over the bumper intake subsystem that can rotate to give the note to
 * either the
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
	private static final double ENCODER_OFFSET = -0.18064158; // Radians
	private static final double OUT_ANGLE_THRESHOLD = 0; // Radians

	private boolean intakeState = false;
	private double currentPosition = 0;
	private double targetPosition = 0;

	private WPI_TalonFX leftRotationMotor = new WPI_TalonFX(3);
	private WPI_TalonFX rightRotationMotor = new WPI_TalonFX(14);
	private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(34);

	private DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(1);

	private PIDController pid = new PIDController(1.0, 0.0, 0.0);

	public IntakeSubsystem() {
		rightRotationMotor.follow(leftRotationMotor);
		rightRotationMotor.setInverted(leftRotationMotor.getInverted());

		intakeMotor.setInverted(true);
		
		rotationEncoder.reset();
	}

	private void setRotationVelocity(double angle) {
		leftRotationMotor.set(ControlMode.Velocity, angle / FALCON_TICKS / 10.0);
	}

	/**
	 * Gets gravitational counter-balancing velocity (setting arm to this should keep it in place)
	 * 
	 * @return Motor velocity in Talon ticks per 100ms
	 */
	private int getGravityVelocity() {
		return 0;
	}

	/**
	 * Sets target position of intake
	 * 
	 * @param angle Angle in degrees (zero is up, CCW is positive)
	 */
	public void setTargetPosition(double angle) {
		targetPosition = angle;
	}

	/**
	 * Get intake arm position
	 * 
	 * @return Angle in radians (zero is up, CCW is positive)
	 */
	public double getPosition() {
		return currentPosition;
	}

	/**
	 * Returns true if intake is out
	 * 
	 * @return Boolean state
	 */
	public boolean getIntakeState() {
		return intakeState;
	}

	/**
	 * Sets the speed of the intake wheels
	 * 
	 * @param speed Percent speed
	 */
	public void setIntakeOutput(double speed) {
		intakeMotor.set(ControlMode.PercentOutput, speed);
	}

	@Override
	public void periodic() {
		currentPosition = -(rotationEncoder.getAbsolutePosition() - rotationEncoder.getPositionOffset()) * 2 * Math.PI + ENCODER_OFFSET;
		// intakeState = getPosition() < OUT_ANGLE_THRESHOLD;

		SmartDashboard.putNumber("encoderPos", currentPosition / DEG);

		// double newVelocity = pid.calculate(currentPosition, targetPosition);
		// setRotationVelocity(Math.max(newVelocity, 0.5));
	}
}
