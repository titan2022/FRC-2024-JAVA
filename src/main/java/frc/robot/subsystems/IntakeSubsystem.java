// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
@SuppressWarnings({"deprecated", "removal"})
public class IntakeSubsystem extends SubsystemBase {
  private static final double GEAR_RATIO = 1;
  // Geared up FalconFX which handles the rotation of the intake
  WPI_TalonFX leftRotationMotor;
  WPI_TalonFX rightRotationMotor;
  CANSparkMax rotationEncoderSpark;
  SparkAbsoluteEncoder rotationEncoder;
  // Simple controller connected to bag motor to control intake of notes
  WPI_TalonSRX wheelsMotorController;
  
  public IntakeSubsystem(int leftRotationPort, int rightRotationPort, int sparkPort, int spinPort) {
    leftRotationMotor = new WPI_TalonFX(leftRotationPort);
    rightRotationMotor = new WPI_TalonFX(rightRotationPort);
    rotationEncoderSpark = new CANSparkMax(sparkPort, CANSparkBase.MotorType.kBrushless);
    rotationEncoder = rotationEncoderSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    wheelsMotorController = new WPI_TalonSRX(spinPort);

    leftRotationMotor.setInverted(true);
  }

  /***
   * Sets the absolute angle of rotation pivot point 
   * @param angle Radians
   */
  public void setRotation(Rotation2d angle) {
    leftRotationMotor.set(ControlMode.Position, angle.getRadians() * GEAR_RATIO / Constants.Unit.FALCON_TICKS);
    rightRotationMotor.set(ControlMode.Position, angle.getRadians() * GEAR_RATIO / Constants.Unit.FALCON_TICKS);
  }

  /***
   * Gets the rotation of the intake in degrees
   * @return Radians
   */
  public Rotation2d getRotation() {
    return new Rotation2d(rightRotationMotor.getSelectedSensorPosition(0) / GEAR_RATIO * Constants.Unit.FALCON_TICKS);
  }

  /***
   * Sets the speed of the intake wheels
   * @param speed Percent
   */
  public void setIntakeVelocity(double speed) {
    wheelsMotorController.set(ControlMode.PercentOutput, speed);
  }
}
