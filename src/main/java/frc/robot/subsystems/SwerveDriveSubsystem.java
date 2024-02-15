package frc.robot.subsystems;

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

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.Constants.FeedForward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.utility.Constants.Unit.*;

public class SwerveDriveSubsystem implements DriveSubsystem {
    // Physical parameters
    public static final double ROBOT_TRACK_WIDTH = 18 * IN; // 0.672; // meters (30 in)
    public static final double ROBOT_LENGTH = 18 * IN; // 0.672; // meter
    public static final double WHEEL_RADIUS = 2 * IN; // 0.0508; // meters (2 in)
    public static final double GEAR_RATIO = 6.12;//8.16;
    public static final double METERS_PER_TICKS = WHEEL_RADIUS * 2 * Math.PI / FALCON_CPR / GEAR_RATIO;

    // Deadbands
    private static final double WHEEL_DEADBAND = 0.01;
    private static final double ROTATOR_DEADBAND = 0.001;

    // CAN ID numbers
    private static final int LEFT_FRONT_MOTOR_PORT = 40;//
    private static final int LEFT_BACK_MOTOR_PORT = 43;
    private static final int RIGHT_FRONT_MOTOR_PORT = 41;
    private static final int RIGHT_BACK_MOTOR_PORT = 42;//

    private static final int LEFT_FRONT_MOTOR_ROTATOR_PORT = 30;
    private static final int LEFT_BACK_MOTOR_ROTATOR_PORT = 33;
    private static final int RIGHT_FRONT_MOTOR_ROTATOR_PORT = 31;//
    private static final int RIGHT_BACK_MOTOR_ROTATOR_PORT = 32;

    private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 50;
    private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 53;
    private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 51;
    private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 52;

    // Rotator encoder offsets
// private static final int FRONT_LEFT_OFFSET = -1024 - 238;//-238;
    private static final int FRONT_LEFT_OFFSET = -1024 - 200;//-238;
    private static final int BACK_LEFT_OFFSET = -3085 - 1024;
    // private static final int FRONT_RIGHT_OFFSET = -1913 + 1024;
    private static final int FRONT_RIGHT_OFFSET = -1810 + 1024;
    private static final int BACK_RIGHT_OFFSET = -1865 + 1024;
    private static final int[] OFFSETS = new int[] { FRONT_LEFT_OFFSET, BACK_LEFT_OFFSET, FRONT_RIGHT_OFFSET,BACK_RIGHT_OFFSET };

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
        public static final int BACK_LEFT = 1;
        public static final int FRONT_RIGHT = 2;
        public static final int BACK_RIGHT = 3;
    }

    private final SimpleMotorFeedforward motorFeedfoward = new SimpleMotorFeedforward(FeedForward.kS, FeedForward.kV, FeedForward.kA);

    // Physical Hardware
    private final WPI_TalonFX[] motors = new WPI_TalonFX[] {
            new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT),
            new WPI_TalonFX(LEFT_BACK_MOTOR_PORT),
            new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT),
            new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
    };
    private final WPI_TalonFX[] rotators = new WPI_TalonFX[] {
            new WPI_TalonFX(LEFT_FRONT_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(LEFT_BACK_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(RIGHT_FRONT_MOTOR_ROTATOR_PORT),
            new WPI_TalonFX(RIGHT_BACK_MOTOR_ROTATOR_PORT)
    };
    private final CANCoder[] encoders = new CANCoder[] {
            new CANCoder(LEFT_FRONT_ENCODER_ROTATOR_PORT),
            new CANCoder(LEFT_BACK_ENCODER_ROTATOR_PORT),
            new CANCoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT),
            new CANCoder(RIGHT_BACK_ENCODER_ROTATOR_PORT)
    };

    // PID slots
    private static final int ROTATOR_SLOT_IDX = 0;
    private static final int MAIN_MOTOR_SLOT_IDX = 0;

    // Kinematics
    // Positions describe the position of each wheel relative to the center of the
    // robot
    private static final Translation2d leftFrontPosition = new Translation2d(-ROBOT_TRACK_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d leftBackPosition = new Translation2d(-ROBOT_TRACK_WIDTH / 2, -ROBOT_LENGTH / 2);
    private static final Translation2d rightFrontPosition = new Translation2d(ROBOT_TRACK_WIDTH / 2, ROBOT_LENGTH / 2);
    private static final Translation2d rightBackPosition = new Translation2d(ROBOT_TRACK_WIDTH / 2, -ROBOT_LENGTH / 2);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontPosition,
            leftBackPosition, rightFrontPosition, rightBackPosition);

    private ChassisSpeeds lastVelocity = new ChassisSpeeds();

    // Locks
    private final TranslationalDrivebase translationalLock = new TranslationalDrivebase() {
        @Override
        public void setVelocity(Translation2d velocity) {
            lastVelocity.vxMetersPerSecond = velocity.getX();
            lastVelocity.vyMetersPerSecond = velocity.getY();
            setVelocities(lastVelocity);
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
            lastVelocity.omegaRadiansPerSecond = omega.getRadians();
            setVelocities(lastVelocity);
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
    public SwerveDriveSubsystem(TalonFXConfiguration mainConfig, TalonFXConfiguration rotatorConfig) {
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

        // Current limits
        // rotatorConfig.supplyCurrLimit = supplyCurrentLimit;
        // mainConfig.supplyCurrLimit = supplyCurrentLimit;
        rotatorConfig.supplyCurrLimit.currentLimit = 12;
        rotatorConfig.supplyCurrLimit.enable = true;
        rotatorConfig.supplyCurrLimit.triggerThresholdCurrent = 12;
        rotatorConfig.supplyCurrLimit.triggerThresholdTime = 0.0;
        mainConfig.supplyCurrLimit.currentLimit = 20;
        mainConfig.supplyCurrLimit.enable = true;
        mainConfig.supplyCurrLimit.triggerThresholdCurrent = 20;
        mainConfig.supplyCurrLimit.triggerThresholdTime = 0.00;
        // mainConfig.closedloopRamp = 0.5;
        SmartDashboard.putNumber("cur lim", 20);

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
            motor.setNeutralMode(NeutralMode.Coast);
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

    public SwerveDriveSubsystem() {
        this(null, null);
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
        SmartDashboard.putNumber("cur lim", curLim);
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
        SmartDashboard.putNumber("Feedforward input Y", feedForwardTicks);
        if (velTicks == 0 && !forceOrient) {
            motors[module].set(ControlMode.Velocity, 0);
            // SmartDashboard.putNumber("set vel " + module, 0);
            return;
        }
        double currTicks = getRotatorEncoderCount(module);
        double targetTicks = CANCODER_CPR / 2 - state.angle.getRadians() * RAD / CANCODER_TICKS;
        double deltaTicks = (targetTicks - currTicks) % CANCODER_CPR;
        if (deltaTicks >= CANCODER_CPR / 2)
            deltaTicks -= CANCODER_CPR;
        else if (deltaTicks <= -CANCODER_CPR / 2)
            deltaTicks += CANCODER_CPR;
        if (deltaTicks >= CANCODER_CPR / 4) {
            deltaTicks -= CANCODER_CPR / 2;
            velTicks *= -1;
            feedForwardTicks *= -1;
        } else if (deltaTicks <= -CANCODER_CPR / 4) {
            deltaTicks += CANCODER_CPR / 2;
            velTicks *= -1;
            feedForwardTicks *= -1;
        }
        // SmartDashboard.putNumber("speed STATE " + module, state.speedMetersPerSecond);
        // SmartDashboard.putNumber("angle STATE " + module, state.angle.getDegrees());
        // SmartDashboard.putNumber("set rot " + module, currTicks + deltaTicks);
        // SmartDashboard.putNumber("set vel " + module, velTicks);
        // SmartDashboard.putNumber("set rot " + module, currTicks + deltaTicks);
        // SmartDashboard.putNumber("cur rot " + module, currTicks);
        // SmartDashboard.putNumber("delta rot" + module, deltaTicks);
        // SmartDashboard.putNumber("target rot " + module, targetTicks);
        motors[module].set(ControlMode.Velocity, velTicks, DemandType.ArbitraryFeedForward, feedForwardTicks);
        rotators[module].set(ControlMode.Position, currTicks + deltaTicks + OFFSETS[module]);
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
        SwerveModuleState[] modules = kinematics.toSwerveModuleStates(inputChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_WHEEL_SPEED);
        for (int i = 0; i < 4; i++) {
            // SwerveModuleState optimized = SwerveModuleState.optimize(modules[i], new Rotation2d(encoders[i].getAbsolutePosition()));
            // applyModuleState(optimized, i);
            applyModuleState(modules[i], i);
        }
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
     * Gets the encoder count for a primary motor.
     * 
     * @param useLeft - Whether to use the left primary motor.
     * @return Encoder count for specified primary motor.
     */
    public double getRotatorEncoderCount(int module) {
        return rotators[module].getSelectedSensorPosition() - OFFSETS[module];
    }

    /**
     * Gets the amount of rotation from a primary motor.
     * 
     * @param useLeft - Whether to use the left primary motor.
     * @return Angle of rotator motor in radians
     */
    public double getRotatorEncoderPosition(int module) {
        return (CANCODER_CPR / 2 - getRotatorEncoderCount(module)) * CANCODER_TICKS / RAD;
    }

    public double getEncoderVelocity(int module) {
        return motors[module].getSelectedSensorVelocity() * METERS_PER_TICKS * 10;
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++)
            states[i] = new SwerveModuleState(getEncoderVelocity(i), new Rotation2d(getRotatorEncoderPosition(i)));
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
}