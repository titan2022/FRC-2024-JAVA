// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.UnsupportedAddressTypeException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * A slam dunker subsystem which can rotate to release notes into the AMP and takes
 * note from the IntakeSubsystem
 */
public class SlamDunkerSubsystem extends SubsystemBase {
  private static final double GEAR_RATIO = 1;
  // Motor to handle the rotation of the slam dunker
  WPI_TalonFX rotatorMotorOne;
  // Follows the first motor
  WPI_TalonFX rotatorMotorTwo;
  // Connected to simple bag motor which is meant to take note from IntakeSubsystem
  // and release into the amp
  WPI_TalonSRX wheelMotorController;
  
  public SlamDunkerSubsystem() {
  }

  public void config() {
    rotatorMotorTwo.follow(rotatorMotorOne);
  }

  /***
   * Sets the speed of the wheels
   * @param speed In percentage from -1 to 1
   */
  public void setWheelVelocity(double speed) {
    wheelMotorController.set(ControlMode.PercentOutput, speed);
  }

  /***
   * Sets the angle of the slam dunker rotation pivot point
   * @param angle Radians
   */
  public void setRotation(Rotation2d angle) {
    rotatorMotorOne.set(ControlMode.Position, angle.getRadians() * gearRatio / Constants.Unit.FALCON_TICKS);
  }

  /***
   * Gets the angle of the slam dunker rotation pivot point
   * @return Radians
   */
  public Rotation2d getRotation() {
    return new Rotation2d(rotatorMotorOne.getSelectedSensorPosition(0) / gearRatio * Constants.Unit.FALCON_TICKS);
  }
}
