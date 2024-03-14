package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
    private final XboxController xbox2 = new XboxController(1);
    private Localizer localizer = new Localizer();
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();

    @Override
    public void robotInit() {
        elevator.leftSpoolMotor.setSelectedSensorPosition(0.0);
        elevator.config();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Current Speed", drive.getTranslational().getVelocity().getNorm());
        SmartDashboard.putNumber("Swerve Heading", localizer.getHeading().getDegrees());

        localizer.step();
    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {
        localizer.setup();

        // Main driver
        drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox1, 2));
        drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox1, Math.PI));

        // Second driver
        shooter.setDefaultCommand(new ShooterControlCommand(shooter, xbox2));
        elevator.setDefaultCommand(new ElevatorControlCommand(elevator, xbox2));
        // Trigger xboxTrigger = new JoystickButton(xbox1, XboxController.Button.kY.value);
        // xboxTrigger.onTrue(new PreSpeakerAlignCommand(drive, localizer, new Rotation2d(0), 0.2 * Math.PI));
    }

    int shooterDir = 1;

    @Override
    public void teleopPeriodic() {
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

        if (!xbox2.getBButton() && !xbox2.getAButton() && !xbox2.getYButton()) {
            indexer.stop();
        }
    }
}