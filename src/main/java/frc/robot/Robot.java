package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.SimpleAutoPlanLeft;
import frc.robot.commands.control.ElevatorControlCommand;
import frc.robot.commands.drive.RotationalDriveCommand;
import frc.robot.commands.drive.TranslationalDriveCommand;
import frc.robot.commands.shooter.ShooterControlCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

public class Robot extends TimedRobot {
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(2);
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    private Localizer localizer = new Localizer(drive, false, 5804); 
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private DataLog log;

    @Override
    public void robotInit() {
        elevator.leftSpoolMotor.setSelectedSensorPosition(0.0);
        elevator.config();
        SmartDashboard.putNumber("swkP", 0.0056);
        SmartDashboard.putNumber("swkI", 0.0);
        SmartDashboard.putNumber("swkD", 0.06);
        SmartDashboard.putNumber("swkF", 0.02);

        DataLogManager.start();
        log = DataLogManager.getLog();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Speed", drive.getTranslational().getVelocity().getNorm());
        SmartDashboard.putNumber("Heading", localizer.getHeading().getDegrees());

        localizer.step();
    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
        new SimpleAutoPlanLeft(drive.getTranslational(), drive.getRotational(), shooter, indexer, intake, elevator, localizer).schedule();
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
        // for (int i = 0; i < drive.motors.length; i++) {
        //     drive.motors[i].config_kP(0, SmartDashboard.getNumber("swkP", 0.0056));
        //     drive.motors[i].config_kI(0, SmartDashboard.getNumber("swkI", 0.0));
        //     drive.motors[i].config_kD(0, SmartDashboard.getNumber("swkD", 0.06));
        //     drive.motors[i].config_kF(0, SmartDashboard.getNumber("swkF", 0.02));
        // }
        localizer.setup();

        // Main driver
        drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox1, 6));
        drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox1, 2.5 * Math.PI));

        // Second driver
        shooter.setDefaultCommand(new ShooterControlCommand(shooter, indexer, xbox2, log));
        elevator.setDefaultCommand(new ElevatorControlCommand(elevator, xbox2));
        // Trigger xboxTrigger = new JoystickButton(xbox1, XboxController.Button.kY.value);
        // xboxTrigger.onTrue(new PreSpeakerAlignCommand(drive, localizer, new Rotation2d(0), 0.2 * Math.PI));
    }

    int shooterDir = 1;

    @Override
    public void teleopPeriodic() {
        if (xbox2.getRightBumper()) {
            shooter.index(0.5);
            indexer.intake();
        } else {
            shooter.index(0);
        }

        if (xbox2.getAButton()) {
            intake.setWheelSpeed(0.45);
            indexer.intake();
        } else if (xbox2.getYButton()) {
            intake.setWheelSpeed(-0.45);
            indexer.reverse();
        } else {
            intake.stop();
        }

        if (xbox2.getBButton()) {
            indexer.reverse();
        }

        if (!xbox2.getBButton() && !xbox2.getAButton() && !xbox2.getYButton() && !xbox2.getRightBumper()) {
            indexer.stop();
        }
    }
}