package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private final WPI_TalonFX MOTOR = new WPI_TalonFX(0);
    private static final double INDEX_PERCENT_OUTPUT = 0.0;
    private static final double AMP_PERCENT_OUTPUT = 0.0;

    public void stop(){
        MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public void index(boolean reverse){
        MOTOR.set(ControlMode.PercentOutput, (reverse ? -1 : 1) * INDEX_PERCENT_OUTPUT);
    }

    public void amp(boolean reverse){
        MOTOR.set(ControlMode.PercentOutput, (reverse ? -1 : 1) * AMP_PERCENT_OUTPUT);
    }

    public IndexerSubsystem(){
        
    }
}
