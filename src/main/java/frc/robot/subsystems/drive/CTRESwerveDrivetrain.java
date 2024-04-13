package frc.robot.subsystems.drive;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CTRESwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.RobotCentric robotOrientedControl = new SwerveRequest.RobotCentric()
      .withDeadband(TunerConstants.MAX_SPEED * 0.1).withRotationalDeadband(TunerConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentric fieldOrientedControl = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.MAX_SPEED * 0.1).withRotationalDeadband(TunerConstants.MAX_SPEED * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private SwerveRequest controlType = fieldOrientedControl;

    public final TranslationalDrivebase translational = new TranslationalDrivebase() {
        @Override
        public void setVelocity(Translation2d velocity) {
            // TODO Auto-generated method stub
            setRobotVelocity(velocity.getX(), velocity.getY());
        }

        @Override
        public Translation2d getVelocity() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
        }
        
        @Override
        public Command translationalDrive(CommandXboxController xbox) {
            return run(() -> {
                double v_x = deadband(xbox.getLeftY());
                double v_y = deadband(xbox.getLeftX());
                double magnitude = Math.sqrt(v_x * v_x + v_y * v_y);
                if (magnitude > TunerConstants.MAX_SPEED) {
                    v_x *= TunerConstants.MAX_SPEED / magnitude;
                    v_y *= TunerConstants.MAX_SPEED / magnitude;
                }
                if (controlType == fieldOrientedControl) {
                    SmartDashboard.putString("Control Mode", "Field");
                    setFieldVelocity(v_x, v_y);
                    applyFieldRequest();
                } else if (controlType == robotOrientedControl) {
                    SmartDashboard.putString("Control Mode", "Robot");
                    setRobotVelocity(v_x, v_y);
                    applyRobotRequest();
                } 
            });
        }
    };


    public final RotationalDrivebase rotational = new RotationalDrivebase() {
        @Override
        public void setRotationalVelocity(Rotation2d omega) {
            setOmega(omega.getRadians());
        }

        @Override
        public Rotation2d getRotationalVelocity() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getRotationalVelocity'");
        }

        @Override
        public Command rotationalDrive(CommandXboxController xbox) {
            return run(() -> {
                setOmega(TunerConstants.MAX_ANGULAR_SPEED * deadband(xbox.getRightX()));
            });
        }
    };

    public CTRESwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        config();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CTRESwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        config();

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CTRESwerveDrivetrain() {
        this(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    }

    /**
     * Must be run after the super constructor in order to config the motors with additional settings
     */
    private void config() {
    }

    public void applySwerveRequest(SwerveRequest requestSupplier) {
        this.setControl(requestSupplier);
    }

    public void applyFieldRequest() {
        this.setControl(fieldOrientedControl);
    }

    public void applyRobotRequest() {
        this.setControl(robotOrientedControl);
    }

    public void setFieldControl() {
        controlType = fieldOrientedControl;
    }
    
    public void setRobotControl() {
        controlType = robotOrientedControl;
    }

    public void setFieldVelocity(double x, double y) {
        fieldOrientedControl.VelocityX = x;
        fieldOrientedControl.VelocityY = y;
    }

    public void setRobotVelocity(double x, double y) {
        robotOrientedControl.VelocityX = x;
        robotOrientedControl.VelocityY = y;
    }

    public void setOmega(double omega) {
        fieldOrientedControl.RotationalRate = omega;
        robotOrientedControl.RotationalRate = omega;
    }

    public void brake() {
        fieldOrientedControl.RotationalRate = 0;
        fieldOrientedControl.VelocityX = 0;
        fieldOrientedControl.VelocityX = 0;
        robotOrientedControl.RotationalRate = 0;
        robotOrientedControl.VelocityX = 0;
        robotOrientedControl.VelocityX = 0;
        setControl(brake);
    }

    private static final double XBOX_DEADBAND = 0.15;
    private double deadband(double input) {
        if (Math.abs(input) > XBOX_DEADBAND)
            return input;
        else 
            return 0;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
