// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
@SuppressWarnings({"deprecated", "removal"})
public class IntakeSubsystem extends SubsystemBase {
  private static final SupplyCurrentLimitConfiguration LIMIT_CONFIG = new SupplyCurrentLimitConfiguration(true, 12, 12, 0 );
  private static final WPI_TalonFX wheelMotor = new WPI_TalonFX(0);
  
  public IntakeSubsystem() {
    config();
  }

  public void config() {
    wheelMotor.setInverted(false);
    wheelMotor.setNeutralMode(NeutralMode.Coast);
    wheelMotor.configSupplyCurrentLimit(LIMIT_CONFIG);
  }

  /***  
   * Sets the speed of the wheels
   * @param speed In percentage from -1 to 1
   */
  public void setWheelSpeed(double speed) {
    wheelMotor.set(ControlMode.PercentOutput, speed);
  }


  /***
   * 
   */
  public void intake() {
    setWheelSpeed(0.5);
  }

  public void stop() {
    setWheelSpeed(0);
  }
}
