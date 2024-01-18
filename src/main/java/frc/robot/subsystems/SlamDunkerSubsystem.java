package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class SlamDunkerSubsystem {
    public SlamDunkerSubsystem() {

    }

    private WPI_TalonSRX intakeMotor = new WPI_TalonSRX(0);
    private WPI_TalonFX armMotor = new WPI_TalonFX(0);

    private Encoder encoder = new Encoder(null, null);

    void Intake() {
        double speed = 10;

        intakeMotor.set(speed);
    }

    void Score() {
        double speed = 10;

        intakeMotor.set(speed);
    }

    void setArm(double speed){
        armMotor.set(speed);
    }
}
