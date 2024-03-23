package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.utility.Constants.Unit.*;

/**
 * Swerve drive class is tuned so that the x is right and y is forward,
 * meaning that we must set the 0 angle as when the wheels are all pointed 
 * to the right. This is due to how the WPI coordinates consider x as forward
 * and y as left, so we must apply a 90 degree rotation to get our x and y
 */
public class SwerveDriveSubsystem implements DriveSubsystem {
    // Physical parameters
    public static final double ROBOT_TRACK_WIDTH = 18 * IN; // 0.672; // meters (30 in)
    public static final double ROBOT_LENGTH = 18 * IN; // 0.672; // meter
    public static final double WHEEL_RADIUS = 2 * IN; // 0.0508; // meters (2 in)
    public static final double GEAR_RATIO = 6.12;//8.16;
    public static final double METERS_PER_TICKS = WHEEL_RADIUS * 2 * Math.PI / FALCON_CPR / GEAR_RATIO;

    // Deadbands
    private static final double WHEEL_DEADBAND = 0.01;
    private static final double ROTATOR_DEADBAND = 0.0001;

    // CAN ID numbers
    private static final int LEFT_FRONT_MOTOR_PORT = 41;
    private static final int RIGHT_FRONT_MOTOR_PORT = 42;
    private static final int LEFT_BACK_MOTOR_PORT = 40;
    private static final int RIGHT_BACK_MOTOR_PORT = 43;

    private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 31;
    private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 32;
    private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 30;
    private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 33;

    private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 51;
    private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 52;
    private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 50;
    private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 53;

    // Rotator encoder offsets
     private static final int FRONT_LEFT_OFFSET = -2930 + 1024 - 0 + 2048; // 1
     private static final int FRONT_RIGHT_OFFSET = -2841 + 1024 + 0 + 2048; // 2
     private static final int BACK_LEFT_OFFSET = -3267 + 1024 - 0 + 2048; // 0
     private static final int BACK_RIGHT_OFFSET = -2143 + 1024 + 0 + 2048; // 3
    //private static final int FRONT_LEFT_OFFSET = -1930 + 1024 + 1024; // 1
    //private static final int BACK_LEFT_OFFSET = -1024 - 190 + 1024; // 0
    //private static final int FRONT_RIGHT_OFFSET = -1835 + 1024 + 1024; // 2
    //private static final int BACK_RIGHT_OFFSET = -3120 - 1024 + 1024; // 3
    private static final int[] OFFSETS = new int[] { FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_LEFT_OFFSET,BACK_RIGHT_OFFSET };

    // Motor inversions
    private static final boolean WHEEL_INVERTED = false;
    private static final boolean ROTATOR_INVERTED = true;

    // Sensor inversions
    private static final boolean WHEEL_PHASE = false;
    private static final boolean ROTATOR_PHASE = false;

    // Physical limits of motors that create translational motion
    private static final double MAX_WHEEL_SPEED = 10 * M / S;

    public static class Module {
        public static final int FRONT_LEFT = 0;
        public static final int FRONT_RIGHT = 1;
        public static final int BACK_LEFT = 2;
        public static final int BACK_RIGHT = 3;
    }

    public static class TranslationalFeedForward 
	{
        // public static final double kS = 0.025;
		public static final double kS = 0.015;
		// public static final double kV = 0.175;
        public static final double kV = 0.175;
		public static final double kA = 0;
    }
    /**
	 * Contains a velocity based PID configuration.
	 * 
	 * @return TalonFX Configuration Object
	 */
	public static TalonFXConfiguration getSwerveDriveTalonDriveConfig() {
		TalonFXConfiguration talon = new TalonFXConfiguration();
		// Add configs here:
		// talon.slot0.kP = 0.05;
		// talon.slot0.kI = 0;
		// talon.slot0.kD = 1.0;
		// talon.slot0.kF = 0;
		// talon.slot0.kP = 0.1;
        talon.slot0.kP = 0;//SmartDashboard.getNumber("swkP", 0.0);
		talon.slot0.kI = 0;//SmartDashboard.getNumber("swkI", 0.0);
		talon.slot0.kD = 0;//SmartDashboard.getNumber("swkD", 0.0);
		talon.slot0.kF = 0;
		talon.slot0.integralZone = 900;
		talon.slot0.allowableClosedloopError = 20;
		talon.slot0.maxIntegralAccumulator = 254.000000;
		return talon;
	}

	/**
	 * Contains a position based PID configuration
	 * 
	 * @return TalonFX Configuration Object
	 */
	public static TalonFXConfiguration getSwerveDriveTalonRotaryConfig() {
		TalonFXConfiguration talon = new TalonFXConfiguration();
		// Add configs here:
		talon.slot0.kP = 0.35;
		talon.slot0.kI = .0053;
		talon.slot0.kD = 0.05;
		talon.slot0.kF = 0;
		talon.slot0.integralZone = 75;
		talon.slot0.allowableClosedloopError = 2;//5;// 217;
		talon.slot0.maxIntegralAccumulator = 5120;
		return talon;
	}

    public static SimpleMotorFeedforward motorFeedfoward = new SimpleMotorFeedforward(TranslationalFeedForward.kS, TranslationalFeedForward.kV, TranslationalFeedForward.kA);

    // Physical Hardware
    public final WPI_TalonFX[] motors = new WPI_TalonFX[] {
            new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT),
            new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT),
            new WPI_TalonFX(LEFT_BACK_MOTOR_PORT),
            new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    };
    public final WPI_TalonFX[] rotators = new WPI_TalonFX[] {
            new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT)
    };
    private final CANCoder[] encoders = new CANCoder[] {
            new CANCoder(LEFT_FRONT_ENCODER_ROTATOR_PORT),
            new CANCoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT),
            new CANCoder(LEFT_BACK_ENCODER_ROTATOR_PORT),
            new CANCoder(RIGHT_BACK_ENCODER_ROTATOR_PORT)
    };

    // PID slots
    private static final int ROTATOR_SLOT_IDX = 0;
    private static final int MAIN_MOTOR_SLOT_IDX = 0;

    // Kinematics
    // Positions describe the position of each wheel relative to the center of the
    // robot
    private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH / 2, -ROBOT_LENGTH / 2);
    private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH / 2, -ROBOT_LENGTH / 2);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontPosition,
            rightFrontPosition, leftBackPosition, rightBackPosition);

    private ChassisSpeeds targetVelocity = new ChassisSpeeds();

    // Locks
    private final TranslationalDrivebase translationalLock = new TranslationalDrivebase() {
        @Override
        public void setVelocity(Translation2d velocity) {
            targetVelocity.vxMetersPerSecond = velocity.getX();
            targetVelocity.vyMetersPerSecond = velocity.getY();
            setVelocities(targetVelocity);
        }

        @Override
        public Translation2d getVelocity() {
            ChassisSpeeds speeds = getVelocities();
            return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        }
    };

    private final RotationalDrivebase rotationalLock = new RotationalDrivebase() {
        @Override
        public void setRotationalVelocity(Rotation2d omega) {
            targetVelocity.omegaRadiansPerSecond = omega.getRadians();
            setVelocities(targetVelocity);
        }

        @Override
        public Rotation2d getRotationalVelocity() {
            ChassisSpeeds speeds = getVelocities();
            return Rotation2d.fromRadians(speeds.omegaRadiansPerSecond);
        }
    };

    /**
     * Creates the swerve drive subsystem
     * 
     * @param mainConfig    Requires PID configuration in slot 0
     * @param rotatorConfig Requires PID configuration in slot 0
     */
    private StructArrayPublisher<SwerveModuleState> publisher;
    public SwerveDriveSubsystem() {
        TalonFXConfiguration mainConfig = getSwerveDriveTalonDriveConfig();
        TalonFXConfiguration rotatorConfig = getSwerveDriveTalonRotaryConfig();

        setFactoryMotorConfig();

        if (mainConfig != null) {
            motors[0].configAllSettings(mainConfig);
        }
        if (rotatorConfig != null) {
            rotators[0].configAllSettings(rotatorConfig);
        }

        mainConfig = new TalonFXConfiguration();
        rotatorConfig = new TalonFXConfiguration();
        motors[0].getAllConfigs(mainConfig);
        rotators[0].getAllConfigs(rotatorConfig);

        // WPILib
        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        // Current limits
        // rotatorConfig.supplyCurrLimit = supplyCurrentLimit;
        // mainConfig.supplyCurrLimit = supplyCurrentLimit;
        rotatorConfig.supplyCurrLimit.currentLimit = 20;
        rotatorConfig.supplyCurrLimit.enable = true;
        rotatorConfig.supplyCurrLimit.triggerThresholdCurrent = 30;
        rotatorConfig.supplyCurrLimit.triggerThresholdTime = 0.01;
        mainConfig.supplyCurrLimit.currentLimit = 30;
        mainConfig.supplyCurrLimit.enable = true;
        mainConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
        mainConfig.supplyCurrLimit.triggerThresholdTime = 0.01;
        // mainConfig.closedloopRamp = 0.5;
        // mainConfig.openloopRamp = 0.5;

        // SmartDashboard.putNumber("cur lim", 20);

        // Deadbands
        rotatorConfig.neutralDeadband = ROTATOR_DEADBAND;
        mainConfig.neutralDeadband = WHEEL_DEADBAND;

        // Apply configurations
        for (WPI_TalonFX motor : motors) {
            motor.configAllSettings(mainConfig);
            motor.setInverted(WHEEL_INVERTED);
            motor.setSensorPhase(WHEEL_PHASE);
            motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
            motor.selectProfileSlot(MAIN_MOTOR_SLOT_IDX, 0);
            motor.setNeutralMode(NeutralMode.Brake);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 50);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
        }
        for (WPI_TalonFX rotator : rotators) {
            rotator.configAllSettings(rotatorConfig);
            rotator.setInverted(ROTATOR_INVERTED);
            rotator.setSensorPhase(ROTATOR_PHASE);
            rotator.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
            rotator.selectProfileSlot(ROTATOR_SLOT_IDX, 0);
            rotator.setNeutralMode(NeutralMode.Brake);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 60);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 50);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 20);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
            rotator.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 5000);
        }
        for (CANCoder encoder : encoders) {
            encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5000);
            encoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 5000);
        }
        for (int i = 0; i < 4; i++)
            rotators[i].configRemoteFeedbackFilter(encoders[i], 0);
    }

    private void setFactoryMotorConfig() {
        for (WPI_TalonFX motor : motors)
            motor.configFactoryDefault();
        for (WPI_TalonFX rotator : rotators)
            rotator.configFactoryDefault();
    }

    private double curLim = 20;

    public void setCurLimit(double limit) {
        SupplyCurrentLimitConfiguration currentLimit = new SupplyCurrentLimitConfiguration(true, limit, limit, 0.05);
        for (WPI_TalonFX motor : motors)
            motor.configSupplyCurrentLimit(currentLimit);
        curLim = limit;
        // SmartDashboard.putNumber("cur lim", curLim);
    }

    public double getCurLimit() {
        return curLim;
    }

    /**
     * Returns the current maximum drive speed in meters per second.
     * 
     * @return Maximum drive speed in meters per second.
     */
    public double getMaxSpeed() {
        return MAX_WHEEL_SPEED;
    }

    private void applyModuleState(SwerveModuleState state, int module, boolean forceOrient) {
        double velTicks = state.speedMetersPerSecond / (10 * METERS_PER_TICKS);
        double feedForwardTicks = motorFeedfoward.calculate(state.speedMetersPerSecond);
        // SmartDashboard.putNumber("Feedforward input Y", feedForwardTicks);
        if (velTicks == 0 && !forceOrient) {
            motors[module].set(ControlMode.Velocity, 0);
            // SmartDashboard.putNumber("set vel " + module, 0);
            return;
        }
        double currentTicks = getRotatorEncoderCount(module);
        //Converts from [-pi, pi] to [0, 2pi] range.
        double targetTicks = state.angle.getRadians() + 2 * Math.PI % (2 * Math.PI);
        //Converts to encoder ticks
        targetTicks /= CANCODER_TICKS;
        //Calculates the difference in ticks on the [0, 2pi] interval
        double deltaTicks = (targetTicks - currentTicks) % CANCODER_CPR;
        double setTicks = currentTicks + deltaTicks;
        // SmartDashboard.putNumber("speed STATE " + module, state.speedMetersPerSecond);
        // SmartDashboard.putNumber("angle STATE " + module, state.angle.getDegrees());
        // SmartDashboard.putNumber("set rot " + module, currTicks + deltaTicks);
        // SmartDashboard.putNumber("set vel " + module, velTicks);
        // SmartDashboard.putNumber("set rot " + module, currTicks + deltaTicks);
        // SmartDashboard.putNumber("cur rot " + module, currTicks);
        // SmartDashboard.putNumber("delta rot" + module, deltaTicks);
        // SmartDashboard.putNumber("target rot " + module, targetTicks);
        motors[module].set(ControlMode.Velocity, velTicks, DemandType.ArbitraryFeedForward, feedForwardTicks);
        // motors[module].set(ControlMode.Velocity, velTicks, DemandType.ArbitraryFeedForward, feedForwardTicks);
        // motors[module].set(ControlMode.Velocity, velTicks);
        rotators[module].set(ControlMode.Position, setTicks);
    }

    private void applyModuleState(SwerveModuleState state, int idx) {
        applyModuleState(state, idx, false);
    }

    public void crossLockWheels() {
        applyModuleState(
                new SwerveModuleState(0, new Rotation2d(-1, 1)), 0, true);
        applyModuleState(
                new SwerveModuleState(0, new Rotation2d(-1, -1)), 1, true);
        applyModuleState(
                new SwerveModuleState(0, new Rotation2d(1, 1)), 2, true);
        applyModuleState(
                new SwerveModuleState(0, new Rotation2d(1, -1)), 3, true);
        // leftFrontPosition, leftBackPosition, rightFrontPosition, rightBackPosition
    }

    /**
     * Sets motor outputs using specified control mode
     * 
     * @param mode             a ControlMode enum
     * @param leftOutputValue  left side output value for ControlMode
     * @param rightOutputValue right side output value for ControlMode
     */
    private void setVelocities(ChassisSpeeds inputChassisSpeeds) {
        SmartDashboard.putNumber("Chassis X", inputChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Y", inputChassisSpeeds.vyMetersPerSecond);
        SwerveModuleState[] modules = kinematics.toSwerveModuleStates(inputChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_WHEEL_SPEED);
        // SmartDashboard.putNumber("FL Speed", modules[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("FL Rot", modules[0].angle.getDegrees());
        // SmartDashboard.putNumber("FR Speed", modules[2].speedMetersPerSecond);
        // SmartDashboard.putNumber("FR Rot", modules[2].angle.getDegrees());
        // SmartDashboard.putNumber("BL Speed", modules[1].speedMetersPerSecond);
        // SmartDashboard.putNumber("BL Rot", modules[1].angle.getDegrees());
        // SmartDashboard.putNumber("BR Speed", modules[3].speedMetersPerSecond);
        // SmartDashboard.putNumber("BR Rot", modules[3].angle.getDegrees());
        for (int i = 0; i < 4; i++) {
            SwerveModuleState optimized = SwerveModuleState.optimize(modules[i], new Rotation2d(encoders[i].getAbsolutePosition()));
            applyModuleState(optimized, i);
            // applyModuleState(modules[i], i);
        }
        publisher.set(modules);
    }

    private void setSwerveStates(SwerveModuleState state) {
        if (state.speedMetersPerSecond > MAX_WHEEL_SPEED)
            state.speedMetersPerSecond = MAX_WHEEL_SPEED;
        for (int i = 0; i < 4; i++) {
            // SwerveModuleState optimized = SwerveModuleState.optimize(modules[i], new Rotation2d(encoders[i].getAbsolutePosition()));
            // applyModuleState(optimized, i);
            applyModuleState(state, i);
        }
    }

    /**
     * Estimates robot velocity from wheel speeds.
     * 
     * @return The estimated robot velocity.
     */
    public ChassisSpeeds getVelocities() {
        SwerveModuleState[] states = getSwerveModuleStates();
        return kinematics.toChassisSpeeds(states);
    }

    // encoder methods

    /**
     * Gets the encoder count for a primary motor offset
     * so that 0 is right according to unit circle angles
     * in terms of can coder ticks or 4096 ticks per rotation.
     * 
     * @param useLeft - Whether to use the left primary motor.
     * @return Encoder count for specified primary motor.
     */
    public double getRotatorEncoderCount(int module) {
        return rotators[module].getSelectedSensorPosition() - OFFSETS[module];
    }

    /**
     * Gets the amount of rotation from a swerve module
     * in radians with 0 as the swerve module pointing right.
     * Aka use unit circle angles
     * 
     * @param useLeft - Whether to use the left primary motor.
     * @return Angle of rotator motor in radians
     */
    public double getRotatorEncoderPosition(int module) {
        return getRotatorEncoderCount(module) * CANCODER_TICKS;
    }

    /**
     * Gets rotation in radians according to robot coordinates
     * or from [-pi, pi] to [0, 2pi]
     * @param module Which swerve module
     * @return Swerve module angle in radians
     */
    public double getRobotRotatorEncoderPosition(int module) {
        if (getRotatorEncoderPosition(module) > Math.PI)
            return getRotatorEncoderPosition(module) - (2 * Math.PI);
        else 
            return getRotatorEncoderCount(module);
    }

    public double getEncoderVelocity(int module) {
        return motors[module].getSelectedSensorVelocity() * METERS_PER_TICKS * 10;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            states[i] = new SwerveModuleState(getEncoderVelocity(i), new Rotation2d(getRobotRotatorEncoderPosition(i)));
        return states;
    }

    @Override
    public TranslationalDrivebase getTranslational() {
        return translationalLock;
    }

    @Override
    public RotationalDrivebase getRotational() {
        return rotationalLock;
    }

    @Override
    public void coast() {
        for (WPI_TalonFX rotator : rotators)
            rotator.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void brake() {
        for (WPI_TalonFX rotator : rotators)
            rotator.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("FL Rot Pos", rotators[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("FL Enc Pos", encoders[0].getPosition());
        SmartDashboard.putNumber("FL Enc AbsPos", encoders[0].getAbsolutePosition());
    }

//     public void setRotatorTest() {
//         int[] adjustments = {
//             (int) SmartDashboard.getNumber("Front Left", 0),
//             (int) SmartDashboard.getNumber("Back Left", 0),
//             (int) SmartDashboard.getNumber("Front Right", 0), 
//             (int) SmartDashboard.getNumber("Back Right", 0)
//         };
//         for (int i = 0; i < 4; ++i) {
//             rotators[i].set(ControlMode.Position, OFFSETS[i] + adjustments[i]);
//         }
//     }

}