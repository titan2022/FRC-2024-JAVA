// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VerticleShooterSubsystem extends SubsystemBase {
  private static final boolean IS_INVERTED = false;
  private static final boolean SENSOR_PHASE = false;
  private static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
  private static final WPI_TalonFX shooterMotorOne;
  private static final WPI_TalonFX shooterMotorTwo;
  private static final WPI_TalonFX loaderMotor;

  public VerticleShooterSubsystem() {
    configMotors();
  }

  private void configMotors() {
    shooterMotorOne.setSensorPhase(SENSOR_PHASE);
    shooterMotorOne.setInverted(IS_INVERTED);
    shooterMotorOne.setNeutralMode(NEUTRAL_MODE);
    shooterMotorOne.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0), 0);
    shooterMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    shooterMotorOne.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    shooterMotorTwo.setSensorPhase(SENSOR_PHASE);
    shooterMotorTwo.setInverted(IS_INVERTED);
    shooterMotorTwo.setNeutralMode(NEUTRAL_MODE);
    shooterMotorTwo.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0), 0);
    shooterMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    shooterMotorTwo.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    shooterMotorTwo.follow(shooterMotorTwo);

    loaderMotor.setSensorPhase(SENSOR_PHASE);
    loaderMotor.setInverted(IS_INVERTED);
    loaderMotor.setNeutralMode(NEUTRAL_MODE);
    loaderMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 20, 0), 0);
    loaderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    loaderMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
  }

  public void setVelocity(double velocity) {
    
  }

  public void loadNote() {

  }
}
