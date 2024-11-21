package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private double kP = 0.5;

    private WPI_TalonSRX m1 = new WPI_TalonSRX(7); // Front left
    private WPI_TalonSRX m2 = new WPI_TalonSRX(1); // Back left
    private WPI_TalonSRX m3 = new WPI_TalonSRX(50); // Front right
    private WPI_TalonSRX m4 = new WPI_TalonSRX(8); // Back Right

    private final XboxController xbox = new XboxController(0);

    @Override
    public void robotInit() {
        m1.setInverted(true);
        m2.setInverted(true);
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("right x", xbox.getRightY());
        SmartDashboard.putNumber("left y", xbox.getLeftY());
        SmartDashboard.putBoolean("A", xbox.getAButtonPressed());
        SmartDashboard.putBoolean("B", xbox.getBButtonPressed());

        m1.set(ControlMode.PercentOutput, kP * xbox.getLeftY());
        m2.set(ControlMode.PercentOutput, kP * xbox.getLeftY());
        m3.set(ControlMode.PercentOutput, kP * xbox.getRightY());
        m4.set(ControlMode.PercentOutput, kP * xbox.getRightY());
    }
}