// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A linkage shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
	private static final double MAX_ANGLE = 65*DEG;
	private static final double MIN_ANGLE = 15*DEG;
	
	// TODO: measure these in CAD
	private static final double SHOOTER_LENGTH = 7.078305*IN; // this is from the shooter pivot to its linkage connection
	private static final double LINKAGE_LONG_ARM_LENGTH = 6.375*IN;
	private static final double LINKAGE_SHORT_ARM_LENGTH = 2.0*IN;
	// the distance of the motor axis from the shooter pivot
	private static final double LINKAGE_PIVOT_DX = 5.270846*IN;
	private static final double LINKAGE_PIVOT_DY = -0.57172*IN;
	private static final double ANGLE_OFFSET = 16.9*DEG;

//   //kP, kI, kD, kF
// 	public static final double[] shooterPID = {0, 0, 0, 0};

// 	public static final double[] shooterFeedForwardParams = {0, 0, 0};

	// SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterFeedForwardParams[0], shooterFeedForwardParams[1], shooterFeedForwardParams[2]);
    public static final PIDController rotationPID = new PIDController(0.1, 0, 0);
    
	public WPI_TalonFX linkageMotor = new WPI_TalonFX(12, "CANivore");
	private WPI_TalonFX topShooterMotor = new WPI_TalonFX(13, "CANivore");
	private WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(5,"CANivore");
	public WPI_TalonFX indexerMotor = new WPI_TalonFX(21, "CANivore");
	public static DutyCycleEncoder linkageEncoder = new DutyCycleEncoder(9);

	// public static boolean INDEX_ON = false;

	public ShooterSubsystem() {
    	config();
	}

	public void setLinkageMotor(double value){
		linkageMotor.set(ControlMode.PercentOutput, value);
	}

	public void config() {
		linkageEncoder.reset();

		bottomShooterMotor.follow(topShooterMotor);
		bottomShooterMotor.setInverted(true);
		bottomShooterMotor.setSensorPhase(true);

		topShooterMotor.setNeutralMode(NeutralMode.Brake);
		bottomShooterMotor.setNeutralMode(NeutralMode.Brake);
		linkageMotor.setNeutralMode(NeutralMode.Brake);
		indexerMotor.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Gets shooter rotation angle
	 * 
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public double getRotation() {
		return linkageEncoder.get()*2.0*Math.PI;
	}

	/**
	 * Sets target angle of the shooter
	 * 
	 * @param angle in radians (zero is parallel to the ground, positive is up)
	 */
	public void setRotation(double angle) {
		SmartDashboard.putNumber("counter2", SmartDashboard.getNumber("counter2", 0.0) + 1);    
		SmartDashboard.putNumber("angle", angle);
		if(angle < MIN_ANGLE || angle > MAX_ANGLE) {
			return;
		}
		// trust me, the math is right
		angle += ANGLE_OFFSET;
		double shooter_x = SHOOTER_LENGTH * Math.cos(angle);
		double shooter_y = SHOOTER_LENGTH * Math.sin(angle);
		double dx = LINKAGE_PIVOT_DX - shooter_x;
		double dy = shooter_y - LINKAGE_PIVOT_DY;
		double d = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    	double theta1 = Math.acos( // Law of Cosines, can also be negative if we want the linkage "inside"
			(-Math.pow(LINKAGE_LONG_ARM_LENGTH, 2) + Math.pow(d, 2) + Math.pow(LINKAGE_SHORT_ARM_LENGTH, 2)) / 
			(2 * d * LINKAGE_SHORT_ARM_LENGTH)
		);
		double theta2 = Math.atan2(dx, dy);
        double angleFromY = theta1 - theta2;
		double targetRotation = Math.PI - angleFromY;
        targetRotation %= 2 * Math.PI;

        if (targetRotation < -Math.PI / 2) {
             targetRotation += 2 * Math.PI;
        } else if (targetRotation > 3 * Math.PI / 2) {
            targetRotation -= 2 * Math.PI;
        }
		double linkageMag = rotationPID.calculate(getRotation(), targetRotation);
        double PID = Math.copySign(Math.min(Math.abs(linkageMag), 10 / FALCON_TICKS), linkageMag);
		linkageMotor.set(ControlMode.Velocity, 3000);
		SmartDashboard.putNumber("target", targetRotation * 180 / Math.PI);
		SmartDashboard.putNumber("PID", PID);
	}

	private double current_velocity = 0.0;

	public void shoot(double velocity) {
		current_velocity = velocity;
		topShooterMotor.set(ControlMode.Velocity, velocity);
		bottomShooterMotor.set(ControlMode.Velocity, velocity);
	}

	private double scale = 1.0;
	public double getShooterVelocity(){
		return current_velocity*10*(4*IN)*scale;
	}

	public void index(double speed) {
		indexerMotor.set(ControlMode.PercentOutput, speed);
	}

	public void holdIndex() {
		indexerMotor.set(ControlMode.Velocity, 0);
	}
}
