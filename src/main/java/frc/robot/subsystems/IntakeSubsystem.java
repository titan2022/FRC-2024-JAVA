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
<<<<<<< HEAD
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
=======
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
>>>>>>> subsystems-testing

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
@SuppressWarnings({"deprecated", "removal"})
public class IntakeSubsystem extends SubsystemBase {
  private static final SupplyCurrentLimitConfiguration LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 12, 12, 0 );

  private static final double GEAR_RATIO = 1;
  private static final WPI_TalonSRX wheelMotorController = new WPI_TalonSRX(15);
  
  public IntakeSubsystem() {
    config();
  }

  public void config() {
    wheelMotorController.setInverted(WHEEL_INVERTED);
    wheelMotorController.setNeutralMode(NeutralMode.Coast);
    wheelMotorController.configSupplyCurrentLimit(LIMIT_CONFIG);
  }

/***
   * Sets the speed of the wheels
   * @param speed In percentage from -1 to 1
   */
  public void setWheelVelocity(double speed) {
    wheelMotorController.set(ControlMode.PercentOutput, speed);
  }
}
