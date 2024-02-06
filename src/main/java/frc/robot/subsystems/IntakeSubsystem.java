// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.channels.UnsupportedAddressTypeException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static final boolean WHEEL_INVERTED = false;
  private static final boolean ROTATOR_SENSOR_PHASE = false;
  private static final SupplyCurrentLimitConfiguration LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 12, 12, 0 );

  private static final double GEAR_RATIO = 1;
  // Motor to handle the rotation of the slam dunker
  public static final DutyCycleEncoder rotationEncoder = new DutyCycleEncoder(1);
  private static final WPI_TalonFX rotatorMotorOne = new WPI_TalonFX(3);
  // Follows the first motor
  private static final WPI_TalonFX rotatorMotorTwo = new WPI_TalonFX(14);
  // Connected to simple bag motor which is meant to take note from IntakeSubsystem
  // and release into the amp
  private static final WPI_TalonSRX wheelMotorController = new WPI_TalonSRX(15);
  
  public IntakeSubsystem() {
    config();
  }

  public void config() {
    rotationEncoder.reset();

    rotatorMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rotatorMotorOne.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rotatorMotorOne.setSensorPhase(ROTATOR_SENSOR_PHASE);
    rotatorMotorOne.setInverted(false);
    rotatorMotorOne.configSupplyCurrentLimit(LIMIT_CONFIG);
    rotatorMotorOne.setNeutralMode(NeutralMode.Brake);

    rotatorMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rotatorMotorTwo.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    rotatorMotorTwo.setInverted(true);
    rotatorMotorTwo.configSupplyCurrentLimit(LIMIT_CONFIG);
    rotatorMotorOne.setNeutralMode(NeutralMode.Brake);
    rotatorMotorTwo.follow(rotatorMotorOne);

    wheelMotorController.setInverted(WHEEL_INVERTED);
    wheelMotorController.setNeutralMode(NeutralMode.Coast);
    wheelMotorController.configSupplyCurrentLimit(LIMIT_CONFIG);
  }

  public void testRotation(double percent)
  {
    rotatorMotorOne.set(ControlMode.PercentOutput, percent);
    // if (Math.abs(percent) > 0.1) {
    //   rotatorMotorOne.set(ControlMode.PercentOutput, Math.copySign(0.1, percent));
    // }
    // else {
    //   rotatorMotorOne.set(ControlMode.PercentOutput, percent);
    // }
  }

    public void testWheelRotation(double percent)
  {
    if (Math.abs(percent) > 0.5) {
      wheelMotorController.set(ControlMode.PercentOutput, Math.copySign(0.5, percent));
    }
    else {
      wheelMotorController.set(ControlMode.PercentOutput, percent);
    }
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
    rotatorMotorOne.set(ControlMode.Position, angle.getRadians() * GEAR_RATIO / Constants.Unit.FALCON_TICKS);
  }

  /***
   * Gets the angle of the slam dunker rotation pivot point
   * @return Radians
   */
  public double getRotation() {
    return rotationEncoder.getAbsolutePosition();
  }
}
