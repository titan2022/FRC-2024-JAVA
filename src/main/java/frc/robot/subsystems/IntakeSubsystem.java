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
 * either the ShooterSubsystem or SlamDunkerSubsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
	private static final double ENCODER_OFFSET = -0.18064158;
	private static final double OUT_ANGLE = 0 * DEG;
	private static final double IN_ANGLE = 180 * DEG;
	private static final double SLAM_DUNKER_ANGLE = 0;
	private static final double ANGLE_THRSHOLD = 5 * DEG;
	
	private static final double INTAKE_VELOCITY = 0.5;

	private static final double MAX_VELOCITY = 10.0 * DEG; // Maximum arm velocity in radians per second

	private PIDController armPID = new PIDController(1.0, 0.0, 0.0);

	public static enum ArmState {
		In,
		Out,
		SlamDunker,
		Rotating
	}

	public static enum IntakeState {
		NoteIn,
		NoteOut,
		Intaking
	}

	private boolean isIntaking = false;
	private IntakeState intakeState = IntakeState.NoteOut;
	private double currentPosition = 0;
	private double targetPosition = 0;

	private WPI_TalonFX leftRotationMotor = new WPI_TalonFX(3);
	private WPI_TalonFX rightRotationMotor = new WPI_TalonFX(14);
	private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(34);
	private DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(1);

	public IntakeSubsystem() {
		rightRotationMotor.follow(leftRotationMotor);
		rightRotationMotor.setInverted(leftRotationMotor.getInverted());

		intakeMotor.setInverted(true);
		
		rotationEncoder.reset();
	}

	/**
	 * Sets arm rotation velocity
	 * 
	 * @param velocity velocity in radians per second
	 */
	private void setRotationVelocity(double velocity) {
		leftRotationMotor.set(ControlMode.Velocity, velocity / FALCON_TICKS / 10.0);
	}

	/**
	 * Sets the velocity of the intake wheels
	 * 
	 * @param velocity Falcon ticks/100ms speed
	 */
	public void setIntakeVelocity(double velocity) {
		intakeMotor.set(ControlMode.Velocity, velocity);
	}

	/**
	 * Returns true if intake limit switch is activated
	 * 
	 * @return boolean state
	 */
	private boolean getIntakeLimitSwitch() {
		return false;
	}

	/**
	 * Sets target position of intake
	 * 
	 * @param angle Angle in radians (zero is up, CCW is positive)
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
	 * Returns arm state enum.
	 * Possible states: in, out, slam dunker, rotating
	 * 
	 * @return IntakeSubsystem.ArmState state
	 */
	public ArmState getArmState() {
		if (Math.abs(getPosition() - IN_ANGLE) < ANGLE_THRSHOLD) {
			return ArmState.In;
		} else if (Math.abs(getPosition() - OUT_ANGLE) < ANGLE_THRSHOLD) {
			return ArmState.Out;
		} else if (Math.abs(getPosition() - SLAM_DUNKER_ANGLE) < ANGLE_THRSHOLD) {
			return ArmState.SlamDunker;
		}
		return ArmState.Rotating;
	}

	/**
	 * Returns intake state.
	 * Possible states: NoteIn, NoteOut, Intaking
	 * 
	 * @return IntakeSubsystem.IntakeState
	 */
	public IntakeState getIntakeState() {
		return intakeState;
	}
	
	/**
	 * Intakes note until it hits the limit switch.
	 * Asynchronous, so call getIntakeState.
	 */
	public void intakeNote() {
		setIntakeVelocity(INTAKE_VELOCITY);
		isIntaking = true;
	}

	@Override
	public void periodic() {
		currentPosition = -(rotationEncoder.getAbsolutePosition() - rotationEncoder.getPositionOffset()) * 2 * Math.PI + ENCODER_OFFSET;
		SmartDashboard.putNumber("Intake Pos", currentPosition / DEG);
		SmartDashboard.putNumber("Target Pos", targetPosition / DEG);

		double newVelocity = armPID.calculate(currentPosition, targetPosition);
		setRotationVelocity(Math.copySign(Math.min(Math.abs(newVelocity), MAX_VELOCITY), newVelocity));

		if (getIntakeLimitSwitch()) {
			intakeState = IntakeState.NoteIn;
			if (isIntaking) {
				setIntakeVelocity(0.0);
			}
		} else {
			if (isIntaking) {
				intakeState = IntakeState.Intaking;
			} else {
				intakeState = IntakeState.NoteOut;
			}
		}
	}
}
