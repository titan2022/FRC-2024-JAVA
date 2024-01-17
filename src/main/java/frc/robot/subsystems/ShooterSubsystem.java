// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * A rotating shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
  // Motor that controls the rotation of the shooter to shoot int the speaker
  WPI_TalonFX rotatorMotor;
  // Controls the speed at which to shoot the note
  WPI_TalonFX wheelMotorOne;
  // Follows the other motor except in opposite direction
  WPI_TalonFX wheelMotorTwo;
  public ShooterSubsystem() {
  }

  /***
   * Sets the velocity of both the shooter motors 
   * @param speed Radians per sec
   */
  public void setShooterVelocity(Rotation2d speed) {

  }

  /***
   * Sets the velocity of both shooter motors
   * @param speed Meters per sec
   */
  public void setShooterVelocity(double speed) {

  }


  /***
   * Sets the absolute angle of the shooter rotation pivot
   * @param angle Radians 
   */
  public void setRotation(Rotation2d angle) {

  }

  /***
   * Gets the rotation of the rotation pivot
   * @return Radians
   */
  public Rotation2d getRotation() {
    throw new UnsupportedOperationException();
  }
}