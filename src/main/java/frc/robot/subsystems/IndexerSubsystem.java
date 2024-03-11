package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    // private static final double INDEX_VELOCITY = 0.0;
    // private static final double AMP_VELOCITY = 0.0;
    // private static final int BREAK_TIMEOUT = 2; // In frames (20ms)

    private WPI_TalonFX motor = new WPI_TalonFX(22);
    // private DigitalInput beamBreakerInput = new DigitalInput(0);

    // private boolean noteStatus = false;
    // private long lastNoteChange = 0;

    public IndexerSubsystem() {
        config();
    }

    public void config() {
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void index(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    // public void amp(boolean reverse) {
    //     motor.set(ControlMode.Velocity, (reverse ? -1 : 1) * AMP_VELOCITY);
    // }

    public boolean hasNote() {
        // return beamBreakerInput.get();
        return false;
    }

    // @Override
    // public void periodic() {
    //     // Beam breaker needs to be in a state for more than its timeout to count (prevents noise)
    //     boolean newNoteStatus = beamBreakerInput.get();
    //     if (newNoteStatus != noteStatus && timer - lastNoteChange > BREAK_TIMEOUT) {
    //         noteStatus = newNoteStatus;
    //         lastNoteChange = timer;
    //     }

    //     timer++;
    // }
}
