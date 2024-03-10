// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
	private static final double MIN_ANGLE = 15.8*DEG;
	
	// TODO: measure these in CAD
	private static final double SHOOTER_LENGTH = 7.078305*IN; // this is from the shooter pivot to its linkage connection
	private static final double LINKAGE_LONG_ARM_LENGTH = 6.375*IN;
	private static final double LINKAGE_SHORT_ARM_LENGTH = 2.0*IN;
	// the distance of the motor axis from the shooter pivot
	private static final double LINKAGE_PIVOT_DX = 5.270846*IN;
	private static final double LINKAGE_PIVOT_DY = -0.57172*IN;
	private static final double ANGLE_OFFSET = 16.9*DEG;
    public static final double ENCODER_OFFSET = 0;

//   //kP, kI, kD, kF
// 	public static final double[] shooterPID = {0, 0, 0, 0};

// 	public static final double[] shooterFeedForwardParams = {0, 0, 0};

	// SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(shooterFeedForwardParams[0], shooterFeedForwardParams[1], shooterFeedForwardParams[2]);
    public static PIDController rotationPID = new PIDController(0, 0, 0);
    
	public WPI_TalonFX linkageMotor = new WPI_TalonFX(12, "CANivore");
	private WPI_TalonFX topShooterMotor = new WPI_TalonFX(13, "CANivore");
	private WPI_TalonFX bottomShooterMotor = new WPI_TalonFX(5,"CANivore");
	public WPI_TalonFX indexerMotor = new WPI_TalonFX(21, "CANivore");
	public static DutyCycleEncoder linkageEncoder = new DutyCycleEncoder(9);


	public ShooterSubsystem() {
    	config();
	}

	public void config() {
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
	 * Gets shooter rotation angle
	 * 
	 * @return Angle in radians (zero is ground, positive is up)
	 */
	public Rotation2d getRotation() {
		return new Rotation2d(linkageEncoder.getAbsolutePosition()*2.0*Math.PI + MIN_ANGLE);
	}

	/**
	 * Sets target angle of the shooter
	 * 
	 * @param angle in radians (zero is parallel to the ground, positive is up)
	 */
	public void setRotation(Rotation2d theta) {
        double angle = theta.getRadians();
		SmartDashboard.putNumber("Counter", SmartDashboard.getNumber("counter2", 0.0) + 1);    
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
		double linkagePIDOutput = rotationPID.calculate(getRotation().getRadians(), targetRotation);
        // double PID = Math.copySign(Math.min(Math.abs(linkageMag), 10 / FALCON_TICKS), linkageMag);
		linkageMotor.set(ControlMode.Velocity, linkagePIDOutput);
		SmartDashboard.putNumber("Target Rotation", targetRotation * 180 / Math.PI);
		SmartDashboard.putNumber("PID Output", linkagePIDOutput);
	}

    public void holdAngle() {
        linkageMotor.set(ControlMode.PercentOutput, 0);
    }


	public void shoot(double velocity) {
		topShooterMotor.set(ControlMode.Velocity, velocity);
	}

	// public double getShooterVelocity(){
	// 	return current_velocity*10*(4*IN)*scale;
	// }

	public void index(double speed) {
		indexerMotor.set(ControlMode.PercentOutput, speed);
	}

	public void holdIndex() {
		indexerMotor.set(ControlMode.PercentOutput, 0);
	}
}
