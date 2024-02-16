// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  public static final double MIN_ELEVATOR_LENGTH = 0;
  public static final double MAX_ELEVATOR_LENGTH = 0;
  WPI_TalonFX climbMotorOne;
  WPI_TalonFX climbMotorTwo;
  WPI_TalonFX beltMotor;

  public ElevatorSubsystem() {

  }

  public void config() {

  }

  public void setElevatorDistance(double distance) {

  }

  public void addElevatorDistance(double distance) {

  }

  public double getCurrentDistance() {
    return 0;
  }

  public void setBeltPosition(double position) {
  }

  public void addBeltPosition(double position) {

  }

  public boolean hasNote() {
    return false;
  }


}
