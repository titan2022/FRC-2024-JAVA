// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * An over the bumper intake subsystem that can rotate to give the note to either the 
 * ShooterSubsystem or SlamDunkerSubsystem.
 */
@SuppressWarnings({"deprecated", "removal"})
public class IntakeSubsystem extends SubsystemBase {
    public static final double INTAKE_SPEED = -0.45;
    public static final WPI_TalonFX wheelMotor = new WPI_TalonFX(19);

    public IntakeSubsystem() {
        config();
    }

    public void config() {
        wheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        wheelMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        wheelMotor.setInverted(true);
        wheelMotor.setNeutralMode(NeutralMode.Brake);
        // wheelMotor.configSupplyCurrentLimit(LIMIT_CONFIG);
    }

    /***  
     * Sets the speed of the wheels
     * @param speed In percentage from -1 to 1
     */
    public void setWheelSpeed(double speed) {
        wheelMotor.set(ControlMode.Velocity, 0, DemandType.ArbitraryFeedForward, speed);
    }

    public double getWheelSpeed() {
        return wheelMotor.getSelectedSensorVelocity();
    }

    public void toggle() {
        if (Math.abs(getWheelSpeed()) <= 0.01) {
            intake();
        } else {
            stop();
        }
    }
    public void intake() {
        setWheelSpeed(INTAKE_SPEED);
    }

    public void reverse() {
        setWheelSpeed(-INTAKE_SPEED);
    }

    public void stop() {
        setWheelSpeed(0);
    }
}
