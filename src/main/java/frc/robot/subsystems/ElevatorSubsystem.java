package frc.robot.subsystems;

import java.net.IDN;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    // TODO: get constants
    private final WPI_TalonFX LEFT_SPOOL_MOTOR = new WPI_TalonFX(0);
    private final WPI_TalonFX RIGHT_SPOOL_MOTOR = new WPI_TalonFX(0);
    private static final WPI_TalonFX INDEXER = new WPI_TalonFX(0);

    private static final double BOTTOM_HEIGHT = 0.0;
    private static final double TOP_HEIGHT = 0.0;
    private static final double SPOOL_RADIUS = 0.0;

    public static final double GRAVITY_OFFSET = 0;
    public static final double ROBOT_WINCH_OFFSET = 0;
    
    // private static final int BOTTOM_ENCODER_TICKS = ENCODER_OFFSET;
    // private static final long TOP_ENCODER_TICKS = Math.round(ENCODER_OFFSET + (TOP_HEIGHT - BOTTOM_HEIGHT) / (2 * Math.PI * SPOOL_RADIUS) * Constants.Unit.FALCON_TICKS);
    
    // private final WPI_TalonFX[] spool_motors = new WPI_TalonFX[] {
    //     LEFT_SPOOL_MOTOR,
    //     RIGHT_SPOOL_MOTOR
    // };

    public static TalonFXConfiguration getSpoolTalonConfig() {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here: 
        talon.slot0.kP = 0.0;
        talon.slot0.kI = 0.0;
        talon.slot0.kD = 0.0;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 75;
        talon.slot0.allowableClosedloopError = 5;
        talon.slot0.maxIntegralAccumulator = 5120;
        return talon;
	}

    public static TalonFXConfiguration getIndexerTalonConfig() {
        TalonFXConfiguration talon = new TalonFXConfiguration();
        // Add configs here: 
        talon.slot0.kP = 0.0;
        talon.slot0.kI = 0.0;
        talon.slot0.kD = 0.0;
        talon.slot0.kF = 0;
        talon.slot0.integralZone = 75;
        talon.slot0.allowableClosedloopError = 5;
        talon.slot0.maxIntegralAccumulator = 5120;
        return talon;
	}


    public ElevatorSubsystem(){
        config();
    }

    public void config() {
        LEFT_SPOOL_MOTOR.configAllSettings(getSpoolTalonConfig());
        RIGHT_SPOOL_MOTOR.configAllSettings(getSpoolTalonConfig());

        RIGHT_SPOOL_MOTOR.follow(LEFT_SPOOL_MOTOR);
        RIGHT_SPOOL_MOTOR.setInverted(true);
        RIGHT_SPOOL_MOTOR.setSensorPhase(true);

        LEFT_SPOOL_MOTOR.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        LEFT_SPOOL_MOTOR.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        INDEXER.configAllSettings(getIndexerTalonConfig());
        INDEXER.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }


    public void setHeight(double height) {
        LEFT_SPOOL_MOTOR.set(ControlMode.Position, height, DemandType.ArbitraryFeedForward, GRAVITY_OFFSET);
    }

    public void winch(double height) {
        LEFT_SPOOL_MOTOR.set(ControlMode.Position, -height - SPOOL_RADIUS, DemandType.ArbitraryFeedForward, -ROBOT_WINCH_OFFSET);
    }

    public void index(double displacement) {
        INDEXER.set(ControlMode.Position, INDEXER.getSelectedSensorPosition() + displacement);
    }

    public void indexAMP() {
        index(0);
    }
}
    public boolean hasNote() {
        return false;
    }

