// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AlignNoteCommand;
import frc.robot.commands.AlignSpeakerCommand;
import frc.robot.commands.MoveElevatorCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.ShootAMPCommand;
import frc.robot.commands.ShootSpeakerCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;
import frc.robot.utility.TeleopListener;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends TimedRobot {
    private final XboxController xbox = new XboxController(0);
    private TeleopListener listener = new TeleopListener(xbox);
    private Localizer localizer = new Localizer();
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(localizer);
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        SmartDashboard.putNumber("Command Test", 0);
        SmartDashboard.putNumber("Subsystem Test", 10);

        SmartDashboard.putNumber("A", 0);
        SmartDashboard.putNumber("B", 0);
        SmartDashboard.putNumber("C", 0);
        SmartDashboard.putNumber("D", 0);
        SmartDashboard.putNumber("E", 0);
        SmartDashboard.putNumber("F", 0);
        
        //These speeds are temporary
        NamedCommands.registerCommand("Shoot Speaker", new ShootSpeakerCommand(60, shooter, indexer, elevator));
        NamedCommands.registerCommand("Algin Speaker", new AlignSpeakerCommand(drive.getTranslational(), drive.getRotational(), localizer));
        NamedCommands.registerCommand("Shoot Amp", new ShootAMPCommand(60, indexer));
        NamedCommands.registerCommand("Algin Amp", new AlignSpeakerCommand(drive.getTranslational(), drive.getRotational(), localizer));
        NamedCommands.registerCommand("Intake Note", new NoteIntakeCommand(elevator,intake));
        NamedCommands.registerCommand("Algin Speaker", new AlignNoteCommand(drive.getTranslational(), drive.getRotational(), localizer));
        
        SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

        autoChooser = AutoBuilder.buildAutoChooser(); 
        SmartDashboard.putData("Auto Mode", autoChooser);
    }   

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        // SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        // SmartDashboard.putNumber("Swerve Angle", localizer.getHeading().getDegrees());
        // SmartDashboard.putNumber("Swerve Rotation Velocity", localizer.getRate());
        SmartDashboard.putNumber("Encoder Angle", shooter.getRotation().getDegrees());
        // SmartDashboard.putNumber("Elevator Current", elevator.LEFT_SPOOL_MOTOR.getOutputCurrent());
        // SmartDashboard.putBoolean("hasNote", elevator.hasNote());
        // SmartDashboard.putBoolean("IsStalling", elevator.isStalling());
        SmartDashboard.putNumber("Encoder Abs", shooter.getAbsoluteRotation());
        localizer.step();
    
    }

    @Override
    public void disabledInit() {
        // drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
        autoChooser.getSelected().schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        localizer.setup();
        listener.enable();
    }

    @Override
    public void teleopPeriodic() {
        listener.execute();
    }

    @Override
    public void testInit() {
        // localizer.setup();
        shooter.rotationPID.setP(SmartDashboard.getNumber("A", 0));
        shooter.rotationPID.setI(SmartDashboard.getNumber("B", 0));
        shooter.rotationPID.setD(SmartDashboard.getNumber("C", 0));
        // shooter.linkageEncoder.reset();

        switch ((int) SmartDashboard.getNumber("Command Test", 0)) {
            case 1:
                CommandScheduler.getInstance().schedule(
                    new TranslationCommand(new Translation2d(SmartDashboard.getNumber("A", 0), SmartDashboard.getNumber("B", 0)),
                        SmartDashboard.getNumber("C", 0), 
                        drive.getTranslational())
                );
                break;
            case 2:
                CommandScheduler.getInstance().schedule(
                    new RotationCommand(new Rotation2d(SmartDashboard.getNumber("A", 0)),
                        new Rotation2d(SmartDashboard.getNumber("B", 0)),  
                        drive.getRotational(),
                        localizer)
                );
                break;
            case 3: 
                    CommandScheduler.getInstance().schedule(
                        new NoteIntakeCommand(elevator, intake)
                    );
                    break;
            case 4: 
                CommandScheduler.getInstance().schedule(
                    new MoveElevatorCommand(true, elevator),
                    new MoveElevatorCommand(false, elevator)
                );
                break;
            case 5: 
                CommandScheduler.getInstance().schedule(
                    new ShootSpeakerCommand(SmartDashboard.getNumber("A", 0), shooter, indexer, elevator)
                );
                break;
            case 6: 
                CommandScheduler.getInstance().schedule(
                    new ShootAMPCommand(SmartDashboard.getNumber("A", 0), indexer)
                );
                break;
            default:
                break;
        }
    }

    @Override
    public void testPeriodic() {
        switch ((int) SmartDashboard.getNumber("Subsystem Test", 0)) {
            case 1:
                drive.getTranslational().setVelocity(new Translation2d(0, 1));
                break;
            case 2:
                drive.getRotational().setRotationalVelocity(new Rotation2d(Math.PI / 4));
                break;
            case 3:
                intake.setWheelSpeed(0.5);
                break;
            case 4:
                intake.intake();
                break;
            case 5:
                intake.stop();
                break;
            case 6:
                double endTimeTwo = Timer.getFPGATimestamp() + 3;
                intake.toggle();
                while (true) {
                if (Timer.getFPGATimestamp() > endTimeTwo) {
                        intake.toggle();
                        break;
                    }
                }
                break;
            case 7:
                shooter.index(0.5);
                break;
            case 8:
                shooter.holdIndex();
                break;
            case 9:
                shooter.shoot(SmartDashboard.getNumber("A", 0));
                break;
            case 10:
                if (xbox.getYButton())
                    shooter.setRotation(Rotation2d.fromDegrees(SmartDashboard.getNumber("D", 0)));
                else if (xbox.getAButton()) 
                    shooter.linkageEncoder.reset();
                else if (xbox.getLeftBumper()) 
                    shooter.linkageMotor.set(ControlMode.PercentOutput, 0.2);
                else if (xbox.getRightBumper())
                    shooter.linkageMotor.set(ControlMode.PercentOutput, -0.2);
                else if (xbox.getXButton())
                    shooter.resetRotation();
                else 
                    shooter.holdAngle();
                // Rotation2d targetAngle = Rotation2d.fromDegrees(SmartDashboard.getNumber("D", 0));
                // shooter.setRotation(targetAngle);

                // if (xbox.getXButton())
                //     shooter.linkageEncoder.reset();
                // if (xbox.getAButton())
                //     shooter.linkageMotor.setNeutralMode(NeutralMode.Coast);
                // else
                //     shooter.linkageMotor.setNeutralMode(NeutralMode.Brake);

                // if (xbox.getYButton())
                //     shooter.setRotation(targetAngle);
                // else
                //     shooter.holdAngle();

                break;
            case 11:
                // elevator.index(0.5);
                break;
            case 12:
                // elevator.stopIndex();
                break;
            case 13:
                elevator.hold();
                break;
            case 14:
                elevator.elevate(SmartDashboard.getNumber("A", 0));
            case 15: 
                elevator.elevate(-SmartDashboard.getNumber("A", 0));;
            default:
                break;
        }
    }
}