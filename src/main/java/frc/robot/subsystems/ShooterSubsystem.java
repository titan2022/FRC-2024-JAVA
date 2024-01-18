// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * A rotating shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
  //10.16cm for the wheel radius
  private static final double WHEEL_RADIUS = 0.1016;
  private static final double GEAR_RATIO = 1;
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
    wheelMotorOne.set(ControlMode.Velocity, speed.getRadians() * WHEEL_RADIUS);
  }

  /***
   * Sets the absolute angle of the shooter rotation pivot
   * @param angle Radians 
   */
  public void setRotation(Rotation2d angle) {
    rotatorMotor.set(ControlMode.Position, angle.getRadians() * GEAR_RATIO / Constants.Unit.FALCON_TICKS);
  }

  /***
   * Gets the rotation of the rotation pivot
   * @return Radians
   */
  public Rotation2d getRotation() {
    return new Rotation2d(rotatorMotor.getSelectedSensorPosition(0) / GEAR_RATIO * Constants.Unit.FALCON_TICKS);
  }
}
