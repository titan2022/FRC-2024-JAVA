// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.UnsupportedAddressTypeException;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A slam dunker subsystem which can rotate to release notes into the AMP and takes
 * note from the IntakeSubsystem
 */
public class SlamDunkerSubsystem extends SubsystemBase {
  // Motor to handle the rotation of the slam dunker
  WPI_TalonFX rotatorMotorOne;
  // Follows the first motor
  WPI_TalonFX rotatorMotorTwo;
  // Connected to simple bag motor which is meant to take note from IntakeSubsystem
  // and release into the amp
  WPI_TalonSRX wheelMotorController;
  
  public SlamDunkerSubsystem() {
  }

  /***
   * Sets the velocity of the slam dunker holder wheels
   * @param velocity Radians per sec
   */
  public void setWheelVelocity(Rotation2d velocity) {

  }

  /***
   * Sets the angle of the slam dunker rotation pivot point
   * @param angle Radians
   */
  public void setRotation(Rotation2d angle) {

  }

  /***
   * Gets the angle of the slam dunker rotation pivot point
   * @return Radians
   */
  public Rotation2d getRotation() {
    throw new UnsupportedOperationException();
  }
}
