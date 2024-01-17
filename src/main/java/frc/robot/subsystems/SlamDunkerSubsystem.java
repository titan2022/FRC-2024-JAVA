// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlamDunkerSubsystem extends SubsystemBase {
  WPI_TalonFX rotatorMotorOne;
  WPI_TalonFX rotatorMotorTwo;
  WPI_TalonSRX wheelMotorController;
  
  public SlamDunkerSubsystem() {
  }
}
