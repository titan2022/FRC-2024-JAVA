// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * A rotating shooter subsystem designed to take notes from the IntakeSubsystem
 * and shoot them into the speaker
 */
public class ShooterSubsystem extends SubsystemBase {
  private static final boolean WHEEL_INVERTED = false;
  private static final boolean WHEEL_SENSOR_PHASE = false;
  private static final boolean ROTATOR_INVERTED = false;
  private static final boolean ROTATOR_SENSOR_PHASE = false;
  private static final SupplyCurrentLimitConfiguration LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 12, 12, 0);

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
    config();
  }

  public void config() {
    wheelMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    wheelMotorOne.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    wheelMotorOne.setSensorPhase(WHEEL_SENSOR_PHASE);
    wheelMotorOne.setInverted(WHEEL_INVERTED);
    wheelMotorOne.configSupplyCurrentLimit(LIMIT_CONFIG);
    wheelMotorOne.setNeutralMode(NeutralMode.Coast);

    wheelMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    wheelMotorTwo.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    wheelMotorTwo.configSupplyCurrentLimit(LIMIT_CONFIG);
    wheelMotorTwo.setNeutralMode(NeutralMode.Coast);
    wheelMotorTwo.follow(wheelMotorOne);

    rotatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rotatorMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rotatorMotor.setSensorPhase(ROTATOR_SENSOR_PHASE);
    rotatorMotor.setInverted(ROTATOR_INVERTED);
    rotatorMotor.configSupplyCurrentLimit(LIMIT_CONFIG);
    rotatorMotor.setNeutralMode(NeutralMode.Brake);
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
