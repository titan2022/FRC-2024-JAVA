// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_TICKS;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
	
	private WPI_TalonFX linkageMotor = new WPI_TalonFX(0);
	private WPI_TalonFX rotatorMotor = new WPI_TalonFX(0);
	private WPI_TalonFX shooterMotor = new WPI_TalonFX(0);

	public ShooterSubsystem() {
    config();
	}

  public void config() {
    rotatorMotor.setInverted(true);

		linkageMotor.setSelectedSensorPosition(0);
		shooterMotor.setSelectedSensorPosition(0);
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
