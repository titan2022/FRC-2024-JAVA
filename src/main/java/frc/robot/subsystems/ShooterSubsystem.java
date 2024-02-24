// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_TICKS;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A linkage shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
	private static final double MAX_ANGLE = 0;
	private static final double MIN_ANGLE = 0;

  //kP, kI, kD, kF
	public static final double[] shooterPID = {0, 0, 0, 0};
	public static final double[] rotatorPID = {0, 0, 0, 0};

	public static final double[] shooterFeedForwardParams = {0, 0, 0};
	public static final double[] rotatorFeedForwardParams = {0, 0, 0};

	SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterFeedForwardParams[0], shooterFeedForwardParams[1], shooterFeedForwardParams[2]);
	ArmFeedforward rotatorFeedforward = new ArmFeedforward(rotatorFeedForwardParams[0], rotatorFeedForwardParams[1], rotatorFeedForwardParams[2]);
	
	public WPI_TalonFX linkageMotor = new WPI_TalonFX(12);
	private WPI_TalonFX topShooterMotor = new WPI_TalonFX(13);
	private WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(5);
	public WPI_TalonFX indexerMotor = new WPI_TalonFX(21);

	public static boolean INDEX_ON = false;

	public ShooterSubsystem() {
    	config();
	}

	public void config() {
		topShooterMotor.setNeutralMode(NeutralMode.Brake);
		bottomShooterMotor.setNeutralMode(NeutralMode.Brake);
		topShooterMotor.setInverted(true);
		bottomShooterMotor.setInverted(true);
		bottomShooterMotor.follow(topShooterMotor);
		linkageMotor.setNeutralMode(NeutralMode.Brake);
		indexerMotor.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Gets shooter rotation angle
	 * 
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return 0; // TODO
	}

	/**
	 * Sets target angle of the intake
	 * 
	 * @param angle in radians (zero is ground, positive is up)
	 */
	public void setRotation(double angle) {

  	}

	public void setRotationalVelocity(double angle) {
		linkageMotor.set(ControlMode.PercentOutput, angle);
	}

	public void holdAngle() {
		linkageMotor.set(ControlMode.PercentOutput, 0);
	}

	public void shoot(double velocity) {
		topShooterMotor.set(ControlMode.PercentOutput, velocity);
	}

	public void setIndexer(double speed) {
		indexerMotor.set(ControlMode.PercentOutput, speed);
	}

	public void holdNote() {
		indexerMotor.set(ControlMode.PercentOutput, 0);
	}
}
