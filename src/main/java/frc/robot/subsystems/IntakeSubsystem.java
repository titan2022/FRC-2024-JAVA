// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
  // Geared up FalconFX which handles the rotation of the intake
  WPI_TalonFX rotationMotor;
  // Simple controller connected to bag motor to control intake of notes
  WPI_TalonSRX wheelsMotorController;
  
  public IntakeSubsystem() {
  }

  /***
   * Sets the absolute angle of rotation pivot point 
   * @param angle Radians
   */
  public void setRotation(Rotation2d angle) {

  }

  /***
   * Gets the rotation of the intake in degrees
   * @return Radians
   */
  public Rotation2d getRotation() {
    throw new UnsupportedOperationException();
  }

  /***
   * Sets the speed of the intake wheels
   * @param speed Radians per sec
   */
  public void setIntakeVelocity(Rotation2d speed) {

  }
}
