package frc.robot.subsystems;

import static frc.robot.utility.Constants.Unit.FALCON_TICKS;
import static frc.robot.utility.Constants.Unit.IN;
import static frc.robot.utility.Constants.Unit.METERS;

import java.net.IDN;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/***
 * Uses stalling mechanism to move
 */
public class ElevatorSubsystem extends SubsystemBase {
    public static final double RAISE_SPEED = 0.5;
    public static final double LOWER_SPEED = -0.5;
    public static double STALL_CURRENT_LIMIT = 15;
    // public static final double INDEXER_SPEED = 0.5;
    // TODO: get constants
    public static final WPI_TalonFX LEFT_SPOOL_MOTOR = new WPI_TalonFX(18, "CANivore");
    public static final WPI_TalonFX RIGHT_SPOOL_MOTOR = new WPI_TalonFX(14, "CANivore");
    // public static final WPI_TalonFX INDEXER = new WPI_TalonFX(0);
    // public static final DigitalInput NOTE_SENSOR = new DigitalInput(0);
    // public static boolean hasNote = false;
    // public static double noteDuration = 0;
    // public static final double BOTTOM_HEIGHT = 0.0;
    // public static final double TOP_HEIGHT = 21.5 * IN / METERS;
    // public static final double SPOOL_RADIUS = 1 * IN / METERS;
    // public static final double GEAR_RATIO = 28;
    // public static final double DISTANCE_PER_TICK = SPOOL_RADIUS * 2 * Math.PI * FALCON_TICKS / GEAR_RATIO;
    // public static final double TOP_HEIGHT_TICKS = (int)(TOP_HEIGHT / (DISTANCE_PER_TICK));
    // public static final int ENCODER_OFFSET = 0;

    // public static final double ROBOT_WINCH_OFFSET = 0;
    public static final double WINCH_SPEED = -0.5;
    
    // private static final int BOTTOM_ENCODER_TICKS = ENCODER_OFFSET;
    // private static final long TOP_ENCODER_TICKS = Math.round(ENCODER_OFFSET + (TOP_HEIGHT - BOTTOM_HEIGHT) / (2 * Math.PI * SPOOL_RADIUS) * FALCON_TICKS);
    

    // private final WPI_TalonFX[] spool_motors = new WPI_TalonFX[] {
    //     LEFT_SPOOL_MOTOR,
    //     RIGHT_SPOOL_MOTOR
    // };

    // public static TalonFXConfiguration getSpoolTalonConfig() {
    //     TalonFXConfiguration talon = new TalonFXConfiguration();
    //     // Add configs here: 
    //     talon.slot0.kP = 0.0;
    //     talon.slot0.kI = 0.0;
    //     talon.slot0.kD = 0.0;
    //     talon.slot0.kF = 0;
    //     talon.slot0.integralZone = 75;
    //     talon.slot0.allowableClosedloopError = 5;
    //     talon.slot0.maxIntegralAccumulator = 5120;
    //     return talon;
	// }

    // public static TalonFXConfiguration getIndexerTalonConfig() {
    //     TalonFXConfiguration talon = new TalonFXConfiguration();
    //     // Add configs here: 
    //     talon.slot0.kP = 0.0;
    //     talon.slot0.kI = 0.0;
    //     talon.slot0.kD = 0.0;
    //     talon.slot0.kF = 0;
    //     talon.slot0.integralZone = 75;
    //     talon.slot0.allowableClosedloopError = 5;
    //     talon.slot0.maxIntegralAccumulator = 5120;
    //     return talon;
	// }


    public ElevatorSubsystem(){
        config();
    }

    public void config() {
        // LEFT_SPOOL_MOTOR.configAllSettings(getSpoolTalonConfig());

        RIGHT_SPOOL_MOTOR.follow(LEFT_SPOOL_MOTOR);
        // RIGHT_SPOOL_MOTOR.setInverted(false);
        // RIGHT_SPOOL_MOTOR.setSensorPhase(false);

        LEFT_SPOOL_MOTOR.setInverted(true);
        LEFT_SPOOL_MOTOR.setSensorPhase(true);


        LEFT_SPOOL_MOTOR.setNeutralMode(NeutralMode.Brake);
        RIGHT_SPOOL_MOTOR.setNeutralMode(NeutralMode.Brake);

        // INDEXER.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        LEFT_SPOOL_MOTOR.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        RIGHT_SPOOL_MOTOR.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }


    // public void setHeight(double height) {
    //     //Two stage elevator 
    //     double motorUnits = height;
    //     //Number of radians
    //     motorUnits /= (SPOOL_RADIUS * GEAR_RATIO);
    //     //Convert to number of FalconTicks
    //     motorUnits /= FALCON_TICKS;

    //     LEFT_SPOOL_MOTOR.set(ControlMode.Position, motorUnits);
    // }

    // public double getHeight() {
    //     double position = LEFT_SPOOL_MOTOR.getSelectedSensorPosition();
    //     position -= ENCODER_OFFSET;
    //     return position * DISTANCE_PER_TICK;
    // }

    public void winch() {
        LEFT_SPOOL_MOTOR.set(ControlMode.PercentOutput, WINCH_SPEED);
    }

    public void elevate(double speed) {
        LEFT_SPOOL_MOTOR.set(ControlMode.PercentOutput, speed);
    }

    // public void raise() {
    //     LEFT_SPOOL_MOTOR.set(ControlMode.PercentOutput, RAISE_SPEED);
    // }

    // public void lower() {
    //     LEFT_SPOOL_MOTOR.set(ControlMode.PercentOutput, LOWER_SPEED);
    // }

    public void hold() {
        LEFT_SPOOL_MOTOR.set(ControlMode.PercentOutput, 0);
    }

    public boolean isStalling() {
        if (LEFT_SPOOL_MOTOR.getOutputCurrent() > STALL_CURRENT_LIMIT) 
            return true;
        else 
            return false;
    }

    public boolean isStalling(double current) {
        if (LEFT_SPOOL_MOTOR.getOutputCurrent() > current) 
            return true;
        else 
            return false;
    }

    // public void index(double speed) {
    // //     INDEXER.set(ControlMode.PercentOutput, speed);
    // // }

    // public boolean hasNote() {
    //     return NOTE_SENSOR.get();
    // } 

    // // @Override
    // public void periodic() {
    //     // Beam breaker needs to be in a state for more than its timeout to count (prevents noise)
    //     if (NOTE_SENSOR.get() && !hasNote) {
    //         noteDuration = Timer.getFPGATimestamp();
    //         noteDuration++;
    //     } 


    // }
}
