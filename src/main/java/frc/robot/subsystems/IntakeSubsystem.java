// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static final boolean ROTATOR_SENSOR_PHASE = false;
  private static final boolean WHEEL_INVERTED = false;
  private static final SupplyCurrentLimitConfiguration LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 12, 12, 0);

  private static final double GEAR_RATIO = 1;
  // Geared up FalconFX which handles the rotation of the intake
  private static final WPI_TalonFX rotationMotorOne = new WPI_TalonFX(3);
  private static final WPI_TalonFX rotationMotorTwo = new WPI_TalonFX(14);
  // Simple controller connected to bag motor to control intake of notes
  public static final WPI_TalonSRX wheelsMotorController = new WPI_TalonSRX(15);
  
  public IntakeSubsystem() {
    config();
  }

  public void config() {
    rotationMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rotationMotorOne.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rotationMotorOne.setSensorPhase(true);
    rotationMotorOne.setInverted(ROTATOR_SENSOR_PHASE);
    rotationMotorOne.configSupplyCurrentLimit(LIMIT_CONFIG);
    rotationMotorOne.setNeutralMode(NeutralMode.Brake);

    rotationMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rotationMotorTwo.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rotationMotorTwo.setSensorPhase(false);
    rotationMotorTwo.setInverted(ROTATOR_SENSOR_PHASE);
    rotationMotorTwo.configSupplyCurrentLimit(LIMIT_CONFIG);
    rotationMotorTwo.setNeutralMode(NeutralMode.Brake);
    rotationMotorTwo.follow(rotationMotorOne);

    // rotationMotorTwo.follow(rotationMotorOne);

    wheelsMotorController.setInverted(WHEEL_INVERTED);
    wheelsMotorController.configSupplyCurrentLimit(LIMIT_CONFIG);
  }

  public void setRotation(Rotation2d angle) {
    rotationMotorOne.set(ControlMode.Position, angle.getRadians() * GEAR_RATIO / Constants.Unit.FALCON_TICKS);
  }

  public void testRotatorOne(double num) {
    rotationMotorOne.set(ControlMode.PercentOutput, num);
  }

  public void testRotatorTwo(double num) {
    rotationMotorTwo.set(ControlMode.PercentOutput, num);
  }

  public void testWheelMotor(double num) {
    wheelsMotorController.set(ControlMode.PercentOutput, num);
  }

  /***
   * Gets the rotation of the intake in degrees
   * @return Radians
   */
  public Rotation2d getRotation() {
    return new Rotation2d(rotationMotorOne.getSelectedSensorPosition(0) / GEAR_RATIO * Constants.Unit.FALCON_TICKS);
  }

  /***
   * Sets the speed of the intake wheels
   * @param speed Percent
   */
  public void setIntakeVelocity(double speed) {
    wheelsMotorController.set(ControlMode.PercentOutput, speed);
  }
}
