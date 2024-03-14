// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.FullShootAMPCommand;
import frc.robot.commands.NoteIntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.RotationCommand;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.ShootAMPCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.commands.control.TeleElevatorCommand;
import frc.robot.commands.control.TeleIndexCommand;
import frc.robot.commands.control.TeleIntakeCommand;
import frc.robot.commands.control.TeleShootCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;
import frc.robot.utility.controller.TeleopListener;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class Robot extends TimedRobot {
    private final XboxController xbox = new XboxController(0);
    private final XboxController xboxTwo = new XboxController(1);
    private Localizer localizer = new Localizer();
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(localizer);
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private SendableChooser<Command> autoChooser;
    // private TeleopListener listener = new TeleopListener(xbox, elevator);
    public double lastTime = 0;

    @Override
    public void robotInit() {
        // SmartDashboard.putNumber("Command Test", 0);
        // SmartDashboard.putNumber("Subsystem Test", 0);

        SmartDashboard.putNumber("A", 10);
        SmartDashboard.putNumber("Angle", 0);

        // SmartDashboard.putNumber("B",0);
        // SmartDashboard.putNumber("C", 0);
        // SmartDashboard.putNumber("D", 0);
        
        //These speeds are temporary
        // NamedCommands.registerCommand("Shoot Speaker", new ShootSpeakerCommand(60, shooter, indexer, elevator));
        // NamedCommands.registerCommand("Algin Speaker", new AlignSpeakerCommand(drive.getTranslational(), drive.getRotational(), localizer));
        // NamedCommands.registerCommand("Shoot Amp", new ShootAMPCommand(60, indexer));
        // NamedCommands.registerCommand("Algin Amp", new AlignSpeakerCommand(drive.getTranslational(), drive.getRotational(), localizer));
        // NamedCommands.registerCommand("Intake Note", new NoteIntakeCommand(indexer, intake));
        // NamedCommands.registerCommand("Algin Speaker", new AlignNoteCommand(drive.getTranslational(), drive.getRotational(), localizer));
        
        // SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

        // autoChooser = AutoBuilder.buildAutoChooser(); 
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }   

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Current Speed", drive.getTranslational().getVelocity().getNorm());
        SmartDashboard.putNumber("Current Angle", shooter.getRotation().getDegrees());

        // SmartDashboard.putNumber("Swerve Angle", localizer.getHeading().getDegrees());
        // SmartDashboard.putNumber("Swerve Rotation Velocity", localizer.getRate());
        // SmartDashboard.putNumber("X Axis", xbox.getLeftX());
        // SmartDashboard.putNumber("Y Axis", xbox.getLeftY());

        // SmartDashboard.putNumber("Encoder Angle", shooter.getRotation().getDegrees());
        // SmartDashboard.putNumber("Elevator Current", elevator.LEFT_SPOOL_MOTOR.getOutputCurrent());
        // SmartDashboard.putBoolean("hasNote", indexer.hasNote());
        // SmartDashboard.putBoolean("canRunEle", elevator.canRun());
        // SmartDashboard.putNumber("Encoder Abs", shooter.getAbsoluteRotation());
        // SmartDashboard.putNumber("Elevator Encoder", elevator.getEncoder());

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
        // listener.enable();
        localizer.setup();

        // drive.getTranslational()
        //         .setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox, 2));
        // drive.getRotational()
        //         .setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox, Math.PI)); 

        // CommandScheduler.getInstance().schedule(
        //     new TeleElevatorCommand(elevator, xboxTwo), 
        //     new TeleIntakeCommand(intake, xboxTwo),
        //     new TeleIndexCommand(indexer, xboxTwo),
        //     new TeleShootCommand(shooter, xboxTwo)
        // );      

        // if (xbox.getRightBumper() && ShootSpeakerCommand.TIMER <= 0) {
        //     CommandScheduler.getInstance().schedule(
        //         new ShootSpeakerCommand(shooter, indexer)
        //     );
        // }
        // MoveElevatorCommand.LOW_SPEED = SmartDashboard.getNumber("A", 0);
        // MoveElevatorCommand.HIGH_SPEED = SmartDashboard.getNumber("A", 0);
        // MoveElevatorCommand.HIGH_SPEED_TIME = SmartDashboard.getNumber("B", 0);
        // MoveElevatorCommand.LOW_SPEED = SmartDashboard.getNumber("C", 0);
        // listener.enable();
        // elevator.LEFT_SPOOL_MOTOR.config_kF(0, SmartDashboard.getNumber("A", 0));
        // elevator.LEFT_SPOOL_MOTOR.config_kI(0, SmartDashboard.getNumber("B", 0));
        // elevator.LEFT_SPOOL_MOTOR.config_kD(0, SmartDashboard.getNumber("C", 0));
        // elevator.RIGHT_SPOOL_MOTOR.config_kF(0, SmartDashboard.getNumber("A", 0));
        // elevator.RIGHT_SPOOL_MOTOR.config_kI(0, SmartDashboard.getNumber("B", 0));
        // elevator.RIGHT_SPOOL_MOTOR.config_kD(0, SmartDashboard.getNumber("C", 0));
    }

    @Override
    public void teleopPeriodic() {
        ShooterSubsystem.OMEGA = Rotation2d.fromDegrees(SmartDashboard.getNumber("A", 0));
        if (xbox.getYButton())
            shooter.setRotation(Rotation2d.fromDegrees(SmartDashboard.getNumber("Angle", 0)));
        else 
            shooter.holdAngle();
            // if (xboxTwo.getRightBumper() && ShootSpeakerCommand.TIMER <= 0) {
        //     CommandScheduler.getInstance().schedule(
        //         new ShootSpeakerCommand(shooter, indexer)
        //     );
        // } 

        // if (xboxTwo.getBButton() && ShootAMPCommand.TIMER <= 0) {
        //     CommandScheduler.getInstance().schedule(
        //         new ShootAMPCommand(indexer)
        //     );
        // } 
        // if (xbox.getYButton()) {
        //     elevator.raise();
        // } else if (xbox.getBButton()) {
        //     elevator.lower();
        // } else if (xbox.getLeftTriggerAxis() > 0.05) {
        //     elevator.elevate(-xbox.getLeftTriggerAxis() * SmartDashboard.getNumber("A", 0));
        // } else {
        //     elevator.hold();
        // }
        // listener.execute();

        // if (xbox.getYButton()){
        //     CommandScheduler.getInstance().schedule(
        //         new TeleElevatorCommand(true, elevator)
        //     );
        // }
        // else if (xbox.getAButton()){
        //     CommandScheduler.getInstance().schedule(
        //         new TeleElevatorCommand(false, elevator)
        //     );
        // }

        // if (xbox.getXButton()) {
        //     shooter.intake();
        //     indexer.intake();
        //     intake.intake();
        // } else if (xbox.getBButton())  {
        //     shooter.reverseIndex();
        //     indexer.reverse();
        //     intake.reverse();
        // } else {
        //     shooter.holdIndex();
        //     indexer.stop();
        //     intake.stop();
        // }

        // if (xbox.getRightBumper()) {
        //     shooter.setLinkageMotor(0.1);
        // } else if (xbox.getLeftBumper()) {
        //     shooter.setLinkageMotor(-0.1);
        // } else {
        //     shooter.holdAngle();
        // }

        // if (xbox.getRightTriggerAxis() > 0.05) {
        //     shooter.shoot(SmartDashboard.getNumber("A", 0));
        // } else 
        //     shooter.shoot(0);

        // if (xbox.getYButton()) {
        //     shooter.intake();
        //     shooter.shoot(0.25);
        // } else {
        //     shooter.shoot(0);
        //     shooter.holdIndex();
        // }

        // if (xbox.getLeftBumper()) {
        //     shooter.setLinkageMotor(0.1);
        // } else if (xbox.getRightBumper()) {
        //     shooter.setLinkageMotor(-0.1);
        // } else 
        //     shooter.holdAngle();

        // if (xbox.getYButton()) {
        //     shooter.shoot(SmartDashboard.getNumber("A", 0));
        // } else 
        //     shooter.shoot(0);
        // if (xbox.getLeftBumper()) {
        //     elevator.elevate(0.2);
        // } else if (xbox.getRightBumper()) {
        //     elevator.elevate(-0.2);
        // } else if (xbox.getYButton())
        //     elevator.raise();
        // else if (xbox.getAButton())
        //     elevator.lower();
        // else 
        //     elevator.hold();

        // if (xbox.getXButton())
        //     elevator.LEFT_SPOOL_MOTOR.setSelectedSensorPosition(0);



        // if (SmartDashboard.getNumber("B", 0) == 1) {
        //     if (xbox.getXButton()) {
        //         CommandScheduler.getInstance().schedule(
        //             new MoveElevatorCommand(true, elevator)
        //         );
        //     } else if (xbox.getBButton()) {
        //         CommandScheduler.getInstance().schedule(
        //             new MoveElevatorCommand(false, elevator)
        //         );
        //     }
        // }

        // if (SmartDashboard.getNumber("B", 0) == 2) {
        //     if (xbox.getXButton()) 
        //         ElevatorSubsystem.LEFT_SPOOL_MOTOR.setSelectedSensorPosition(0);
        // }

        

        // if (xbox.getXButton())
        //     indexer.index(0.6);
        // else if (xbox.getBButton())
        //     indexer.index(-0.6);
        // else
        //     indexer.stop();

        // if (xbox.getXButton()) {
        //     // lastTime = Timer.getFPGATimestamp();
        //     CommandScheduler.getInstance().schedule(
        //         new MoveElevatorCommand(true, elevator)
        //     );
        // } else if (xbox.getBButton()) {
        //     CommandScheduler.getInstance().schedule(
        //         new MoveElevatorCommand(false, elevator)
        //     );
        // }




        // if (Math.abs(xbox.getLeftTriggerAxis()) > 0.01){
        //     SmartDashboard.putBoolean("Two", true);
        //     elevator.elevate(SmartDashboard.getNumber("A", 0) * xbox.getLeftTriggerAxis());
        // }
        // else if (Math.abs(xbox.getRightTriggerAxis()) > 0.01)
        //     elevator.elevate(-SmartDashboard.getNumber("A", 0) * xbox.getRightTriggerAxis());
    }

    @Override
    public void testInit() {
        localizer.setup();
        // MoveElevatorCommand.HIGH_SPEED = SmartDashboard.getNumber("A", 0);
        // MoveElevatorCommand.HIGH_SPEED_TIME = SmartDashboard.getNumber("B", 0);
        // MoveElevatorCommand.LOW_SPEED = SmartDashboard.getNumber("A", 0);

        int command = (int) SmartDashboard.getNumber("Command Test", 0);
        // SmartDashboard.putNumber("Command Test", command);
        // if (command == 1) {
        //     CommandScheduler.getInstance().schedule(
        //         new TeleElevatorCommand(true, elevator)
        //     );
        // } else if (command == 2) {
        //     CommandScheduler.getInstance().schedule(
        //         new TeleElevatorCommand(false, elevator)
        //     );
        // } else if (command == 3) {
        //     CommandScheduler.getInstance().schedule(
        //         new NoteIntakeCommand(indexer, intake)
        //     );
        // } else if (command == 4) {
        //     Rotation2d rotate = Rotation2d.fromDegrees(SmartDashboard.getNumber("A", 0));
        //     CommandScheduler.getInstance().schedule(
        //         new RotateShooterCommand(rotate, shooter)
        //     );
        // }
        // CommandScheduler.getInstance().schedule(
        //     new MoveElevatorCommand(true, elevator)
        // );
        // ElevatorSubsystem.STALL_CURRENT_LIMIT = SmartDashboard.getNumber("ELE CUR LIMIT", 0);
        // shooter.linkageEncoder.reset();
        // boolean up = (SmartDashboard.getNumber("Up", 0) == 1);

        // CommandScheduler.getInstance().schedule(
        //     new MoveElevatorCommand(elevator)
        //     //new FullShootAMPCommand(elevator, indexer)
        //             // new MoveElevatorCommand(false, elevator)
        // );

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
        //                 new NoteIntakeCommand(indexer, intake)
        //             );
        //             break;
        //     case 4: 
        //         CommandScheduler.getInstance().schedule(
        //             new MoveElevatorCommand(true, elevator)
        //             // new MoveElevatorCommand(false, elevator)
        //         );
        //         break;
        //     case 5: 
        //         // CommandScheduler.getInstance().schedule(
        //         //     new ShootSpeakerCommand(SmartDashboard.getNumber("A", 0), shooter, indexer, elevator)
        //         // );
        //         break;
        //     case 6: 
        //         CommandScheduler.getInstance().schedule(
        //             new ShootAMPCommand(SmartDashboard.getNumber("A", 0), indexer)
        //         );
        //         break;
        //     case 7: 
        //         CommandScheduler.getInstance().schedule(
        //             new FullShootAMPCommand(elevator, indexer)
        //         );
        //         break;
        //     default:
        //         break;
        
    }

    @Override
    public void testPeriodic() {
        if (xbox.getLeftBumper()){
            elevator.elevate(0.2);
        }
        else if (xbox.getRightBumper()) {
            elevator.elevate(-0.2);
        } else {
            elevator.hold();
        }
        
        if (xbox.getXButton())
            elevator.resetEncoder();

        // if (xbox.getYButton()) {
        //     CommandScheduler.getInstance().schedule(
        //         new MoveElevatorCommand(elevator)
        //     );
        // }
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
            // case 7:
            //     shooter.index(0.5);
            //     break;
            // case 8:
            //     shooter.holdIndex();
            //     break;
            // case 9:
            //     shooter.shoot(SmartDashboard.getNumber("A", 0));
            //     break;
            // case 10:
            //     if (xbox.getYButton())
            //         shooter.setRotation(Rotation2d.fromDegrees(SmartDashboard.getNumber("D", 0)));
            //     else if (xbox.getAButton()) 
            //         shooter.linkageEncoder.reset();
            //     else if (xbox.getLeftBumper()) 
            //         shooter.linkageMotor.set(ControlMode.PercentOutput, 0.2);
            //     else if (xbox.getRightBumper())
            //         shooter.linkageMotor.set(ControlMode.PercentOutput, -0.2);
            //     else if (xbox.getXButton())
            //         shooter.resetRotation();
            //     else 
            //         shooter.holdAngle();
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
                // break;
        //     case 11:
        //         if (xbox.getYButton())
        //             indexer.index(0.5);
        //         else if (xbox.getAButton())
        //             indexer.index(-0.5);
        //         else
        //             indexer.stop();
        //         break;
        //     case 12:
        //         SmartDashboard.putBoolean("One", true);

        //         if (Math.abs(xbox.getLeftTriggerAxis()) > 0.01){
        //             SmartDashboard.putBoolean("One", true);
        //             elevator.elevate(SmartDashboard.getNumber("A", 0) * xbox.getLeftTriggerAxis());
        //         }
        //         else if (Math.abs(xbox.getRightTriggerAxis()) > 0.01)
        //             elevator.elevate(-SmartDashboard.getNumber("A", 0) * xbox.getRightTriggerAxis());
        //         else 
        //             elevator.hold();
        //         break;
        //     default:
        //         break;
        // }
    }
}