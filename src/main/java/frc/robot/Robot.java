package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.control.MoveElevatorCommand;
import frc.robot.commands.control.NoteIntakeCommand;
import frc.robot.commands.drive.RotationalDriveCommand;
import frc.robot.commands.drive.TranslationalDriveCommand;
import frc.robot.commands.shooter.RotateShooterCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

public class Robot extends TimedRobot {
    private final XboxController xbox = new XboxController(0);
    // private TeleopListener listener = new TeleopListener(xbox);
    private Localizer localizer = new Localizer();
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(localizer);
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private SendableChooser<Command> autoChooser;
    public double lastTime = 0;

    @Override
    public void robotInit() {
        elevator.leftSpoolMotor.setSelectedSensorPosition(0.0);
        SmartDashboard.putNumber("Command Test", 0);
        SmartDashboard.putNumber("Subsystem Test", 0);
        SmartDashboard.putString("Test", "0");
        SmartDashboard.putNumber("A", 0);
        SmartDashboard.putNumber("B", 0);
        SmartDashboard.putNumber("C", 0);
        SmartDashboard.putNumber("D", 0);
        SmartDashboard.putNumber("E", 0);
        SmartDashboard.putNumber("F", 0);
        SmartDashboard.putNumber("G", 0);
        SmartDashboard.putNumber("H", 0);
        SmartDashboard.putNumber("I", 0);        
        SmartDashboard.putNumber("J", 0);
        SmartDashboard.putNumber("K", 0);
        SmartDashboard.putNumber("L", 0);
        SmartDashboard.putNumber("Encoder_Offset", 0);
        SmartDashboard.putNumber("top_height",0.5);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Current Speed", drive.getTranslational().getVelocity().getNorm());
        SmartDashboard.putNumber("Swerve Angle", localizer.getHeading().getDegrees());
        SmartDashboard.putNumber("Swerve Rotation Velocity", localizer.getRate());
        SmartDashboard.putNumber("X Axis", xbox.getLeftX());
        SmartDashboard.putNumber("Y Axis", xbox.getLeftY());

        // SmartDashboard.putNumber("Encoder Angle", shooter.getRotation().getDegrees());
        SmartDashboard.putNumber("Elevator Current", elevator.leftSpoolMotor.getOutputCurrent());
        SmartDashboard.putBoolean("hasNote", indexer.hasNote());
        SmartDashboard.putBoolean("canRunEle", elevator.canRun());
        // SmartDashboard.putNumber("Encoder Abs", shooter.getAbsoluteRotation());
        SmartDashboard.putNumber("Elevator Encoder", elevator.getEncoder());
        SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
        localizer.step();
    
    }

    @Override
    public void disabledInit() {
        // drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
        // autoChooser.getSelected().schedule();
        // CommandScheduler.getInstance().schedule(
        //     new TranslationCommand(new Translation2d(0, 3), 1, drive.getTranslational())
        // );
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // if (xbox.getXButton()) {
        //     CommandScheduler.getInstance().schedule(
        //         new MoveElevatorCommand(true, elevator)
        //     );
        // } else if (xbox.getBButton()) {
        //     CommandScheduler.getInstance().schedule(
        //         new MoveElevatorCommand(false, elevator)
        //     );
        // }
    }

    @Override
    public void teleopInit() {
        drive.getTranslational()
                .setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox, 2));
        drive.getRotational()
                .setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox, Math.PI));
        localizer.setup();
        // MoveElevatorCommand.HIGH_SPEED = SmartDashboard.getNumber("A", 0);
        // MoveElevatorCommand.HIGH_SPEED_TIME = SmartDashboard.getNumber("B", 0);
        // MoveElevatorCommand.LOW_SPEED = SmartDashboard.getNumber("C", 0);
        // listener.enable();
        elevator.config();
        // elevator.RIGHT_SPOOL_MOTOR.config_kF(0, SmartDashboard.getNumber("A", 0));
        // elevator.RIGHT_SPOOL_MOTOR.config_kI(0, SmartDashboard.getNumber("B", 0));
        // elevator.RIGHT_SPOOL_MOTOR.config_kD(0, SmartDashboard.getNumber("C", 0));
    }

    @Override
    public void teleopPeriodic() {
        // SmartDashboard.putNumber("counter", SmartDashboard.getNumber("counter", 0.0) + 1);    
        // SmartDashboard.putNumber("cur_test", (int)(SmartDashboard.getNumber("Test", 0)));
        // switch((int)(SmartDashboard.getNumber("Test", 0))){
        //     case 0:
            //     SmartDashboard.putNumber("linkage_angle", shooter.getRotation()*180/Math.PI);
            //     listener.execute();
            //     if(xbox.getAButton()){
            //         shooter.setRotation(SmartDashboard.getNumber("D", 0.0) * Math.PI / 180.0);
            //     }
            //     else {
            //         if(xbox.getLeftBumper()){
            //             shooter.linkageMotor.set(ControlMode.Velocity, 6000);
            //             // shooter.linkageMotor.set(ControlMode.PercentOutput, 0.1);
            //         } else if(xbox.getRightBumper()){
            //             shooter.linkageMotor.set(ControlMode.Velocity, -6000);
            //         } else {
            //             shooter.linkageMotor.set(ControlMode.PercentOutput, 0.0);
            //         }
            //     }

            //     shooter.rotationPID.setP(SmartDashboard.getNumber("A", 0));
            //     shooter.rotationPID.setI(SmartDashboard.getNumber("B", 0));
            //     shooter.rotationPID.setD(SmartDashboard.getNumber("C", 0));
            //     break;
            // case 1:
        SmartDashboard.putNumber("dashboard_counter", SmartDashboard.getNumber("dashboard_counter", 0.0) + 1);
        SmartDashboard.putNumber("encoder", elevator.leftSpoolMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("error", elevator.leftSpoolMotor.getClosedLoopError());
        SmartDashboard.putNumber("input", elevator.leftSpoolMotor.getSupplyCurrent());
        SmartDashboard.putNumber("output", elevator.leftSpoolMotor.getStatorCurrent());
        if(xbox.getAButton()){
            elevator.setHeight(1);
        } else if(xbox.getBButton()){
            elevator.setHeight(0);
        } else {
            elevator.leftSpoolMotor.set(ControlMode.PercentOutput, 0.2*(xbox.getLeftTriggerAxis() - xbox.getRightTriggerAxis()));
        }
        if(xbox.getXButton()){
            intake.intake();
            indexer.intake();
        } else {
            intake.stop();
        }
        if(xbox.getRightBumper()){
            elevator.leftSpoolMotor.setSelectedSensorPosition(0.0);
        }
        if(xbox.getLeftBumper()){
            indexer.reverse();
        }
        if(!xbox.getLeftBumper() && !xbox.getXButton()){
            indexer.stop();
        }

                // break;
        // }

    }

    @Override
    public void testInit() {
        // localizer.setup();
        // switch ((int) SmartDashboard.getNumber("Command Test", 0)) {
        //     case 1:
        //         CommandScheduler.getInstance().schedule(
        //             new TranslationCommand(new Translation2d(SmartDashboard.getNumber("A", 0), SmartDashboard.getNumber("B", 0)),
        //                 SmartDashboard.getNumber("C", 0), 
        //                 drive.getTranslational())
        //         );
        //         break;
        //     case 2:
        //         CommandScheduler.getInstance().schedule(
        //             new RotationCommand(new Rotation2d(SmartDashboard.getNumber("A", 0)), 
        //                 new Rotation2d(SmartDashboard.getNumber("B", 0)), 
        //                 drive.getRotational(),
        //                 localizer)
        //         );
        //         break;
        //     case 3: 
        //             CommandScheduler.getInstance().schedule(
        //                 new NoteIntakeCommand(elevator, intake)
        //             );
        //             break;
        //     case 4: 
        //         CommandScheduler.getInstance().schedule(
        //             new MoveElevatorCommand(true, elevator),
        //             new MoveElevatorCommand(false, elevator)
        //         );
        //         break;
        //     case 5: 
        //         CommandScheduler.getInstance().schedule(
        //             new ShootSpeakerCommand(SmartDashboard.getNumber("A", 0), shooter, elevator)
        //         );
        //         break;
        //     case 6: 
        //         CommandScheduler.getInstance().schedule(
        //             new ShootAMPCommand(SmartDashboard.getNumber("A", 0), elevator)
        //         );
        //         break;
        //     default:
        //         break;
        // }
    }

    // @Override
    // public void testPeriodic() {
    //     switch ((int) SmartDashboard.getNumber("Subsystem Test", 0)) {
    //         case 1:
    //             drive.getTranslational().setVelocity(new Translation2d(0, 1));
    //             break;
    //         case 2:
    //             drive.getRotational().setRotationalVelocity(new Rotation2d(Math.PI / 4));
    //             break;
    //         case 3:
    //             intake.setWheelSpeed(0.5);
    //             break;
    //         case 4:
    //             intake.intake();
    //             break;
    //         case 5:
    //             intake.stop();
    //             break;
    //         case 6:
    //             double endTimeTwo = Timer.getFPGATimestamp() + 3;
    //             intake.toggle();
    //             while (true) {
    //             if (Timer.getFPGATimestamp() > endTimeTwo) {
    //                     intake.toggle();
    //                     break;
    //                 }
    //             }
    //             break;
    //         case 7:
    //             shooter.index(0.5);
    //             break;
    //         case 8:
    //             shooter.holdIndex();
    //             break;
    //         case 9:
    //             shooter.shoot(SmartDashboard.getNumber("A", 0));
    //             break;
    //         case 10:
    //             shooter.rotationPID.setP(SmartDashboard.getNumber("A", 0));
    //             shooter.rotationPID.setI(SmartDashboard.getNumber("B", 0));
    //             shooter.rotationPID.setD(SmartDashboard.getNumber("C", 0));
    //             Rotation2d targetAngle = Rotation2d.fromDegrees(SmartDashboard.getNumber("D", 0));

    //             if (shooter.getRotation().getDegrees() < targetAngle.getDegrees())
    //                 shooter.setRotation(targetAngle);
    //             else 
    //                 shooter.holdAngle();
    //             break;
    //         case 11:
    //             elevator.index(0.5);
    //             break;
    //         case 12:
    //             elevator.stopIndex();
    //             break;
    //         case 13:
    //             elevator.hold();
    //             break;
    //         case 14:
    //             elevator.elevate(SmartDashboard.getNumber("A", 0));
    //         case 15: 
    //             elevator.elevate(-SmartDashboard.getNumber("A", 0));;
    //         default:
    //             break;
    //     }
    // }
}