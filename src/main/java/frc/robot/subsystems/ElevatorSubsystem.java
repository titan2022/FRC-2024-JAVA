package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    // TODO: get constants
    private final WPI_TalonFX LEFT_SPOOL_MOTOR = new WPI_TalonFX(0);
    private final WPI_TalonFX RIGHT_SPOOL_MOTOR = new WPI_TalonFX(0);
    private final WPI_TalonFX WINCH_MOTOR = new WPI_TalonFX(0);
    private static final boolean LEFT_IS_INVERTED = false;
    private static final boolean RIGHT_IS_INVERTED = false;

    private static final double BOTTOM_HEIGHT = 0.0;
    private static final double TOP_HEIGHT = 0.0;
    private static final double SPOOL_RADIUS = 0.0;
    private static final int ENCODER_OFFSET = 0;
    
    private static final int BOTTOM_ENCODER_TICKS = ENCODER_OFFSET;
    private static final long TOP_ENCODER_TICKS = Math.round(ENCODER_OFFSET + (TOP_HEIGHT - BOTTOM_HEIGHT) / (2 * Math.PI * SPOOL_RADIUS) * Constants.Unit.FALCON_TICKS);
    
    
    private final WPI_TalonFX[] spool_motors = new WPI_TalonFX[] {
        LEFT_SPOOL_MOTOR,
        RIGHT_SPOOL_MOTOR
    };

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

    public void extendElevator(){
        LEFT_SPOOL_MOTOR.set(ControlMode.Position, TOP_ENCODER_TICKS);
    }

    public void retractElevator(){
        LEFT_SPOOL_MOTOR.set(ControlMode.Position, BOTTOM_ENCODER_TICKS);
    }

    public ElevatorSubsystem(){
        for(WPI_TalonFX motor : spool_motors){
            motor.configAllSettings(getSpoolTalonConfig());
            // IDK if this is needed
            // motor.setInverted(WHEEL_INVERTED);
            // motor.setSensorPhase(WHEEL_PHASE);
            // motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            // motor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
            // motor.setNeutralMode(NeutralMode.Coast);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 50);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
            // motor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
        }        
    }

}
