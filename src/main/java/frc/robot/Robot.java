package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import static frc.robot.utility.Constants.Unit.*;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoIntakeCommand;
import frc.robot.commands.auto.HoldShooterRevCommand;
import frc.robot.commands.auto.ShootCommand;
import frc.robot.commands.control.ElevatorControlCommand;
import frc.robot.commands.control.IntakeIndexerControlCommand;
import frc.robot.commands.drive.AlignSpeakerCommand;
import frc.robot.commands.drive.RotationalDriveCommand;
import frc.robot.commands.drive.TranslationalDriveCommand;
import frc.robot.commands.shooter.ShooterAlignSpeakerCommand;
import frc.robot.commands.shooter.ShooterControlCommand;
import frc.robot.commands.shooter.ShooterSpeakerAlgCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

public class Robot extends TimedRobot {
    private final XboxController xbox1 = new XboxController(0);
    private final XboxController xbox2 = new XboxController(1);
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    private Localizer localizer = new Localizer(drive, true, 5804); 
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private LEDSubsystem led = new LEDSubsystem();
    private DataLog log;
    private Command auto;

    @Override
    public void robotInit() {
        localizer.setup();
        // SmartDashboard.putNumber("R", 0);
        // SmartDashboard.putNumber("G", 0);
        // SmartDashboard.putNumber("B", 0);
        SmartDashboard.putNumber("kP", 0.35);
        SmartDashboard.putNumber("kI", 0.0035);
        SmartDashboard.putNumber("kD", 0.05);

        SmartDashboard.putNumber("Speaker Distance", 0);
        // SmartDashboard.putNumber("Bot Shooter", 0.01);
        SmartDashboard.putNumber("Tar Shoot Speed", 0);
        SmartDashboard.putNumber("Rotator Static", 0);
        AutoBuilder.configureHolonomic(
            localizer::getDisplacementPose2d,
            localizer::resetPose2d,
            drive::getVelocities,
            drive::setVelocities, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(7.5, 0, 1),
                new PIDConstants(.5, 0, 0), 
                drive.getMaxSpeed(), 
                12.3743687 * IN , 
                new ReplanningConfig()
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            } , drive
        );
        NamedCommands.registerCommand("Intake", new AutoIntakeCommand(indexer, intake, shooter));
        NamedCommands.registerCommand("Rev", new HoldShooterRevCommand(16, shooter));
        NamedCommands.registerCommand("Shoot", new ShootCommand(shooter, indexer));
        NamedCommands.registerCommand("Align", new ShooterAlignSpeakerCommand(16 * 1.15, shooter, localizer));
    //     elevator.leftSpoolMotor.setSelectedSensorPosition(0.0);
    //     elevator.config();
    //     SmartDashboard.putNumber("swkP", 0.0056);
    //     SmartDashboard.putNumber("swkI", 0.0);
    //     SmartDashboard.putNumber("swkF", 0.02);
    //     SmartDashboard.putNumber("swkD", 0.06);
    //     SmartDashboard.putNumber("A", 0.0775);
    //     SmartDashboard.putNumber("D", 30);
    //     SmartDashboard.putNumber("E", 0.005);
    //     SmartDashboard.putNumber("F", -2.55);
        // SmartDashboard.putNumber("Delta FL", 0);
        // SmartDashboard.putNumber("Delta FR", 0);
        // SmartDashboard.putNumber("Delta BL", 0);
        // SmartDashboard.putNumber("Delta BR", 0);

        DataLogManager.start();
        log = DataLogManager.getLog();
        auto = new PathPlannerAuto("Test");

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Y Velocity", drive.getTranslational().getVelocity().getY());
        // Translation2d rotatedVelocity = drive.getTranslational().getVelocity().rotateBy(localizer.getHeading());
        // SmartDashboard.putNumber("Rotated vx", rotatedVelocity.getX());
        // SmartDashboard.putNumber("Rotated vy", rotatedVelocity.getY());
        // SmartDashboard.putNumber("Speed", drive.getTranslational().getVelocity().getNorm());
        SmartDashboard.putNumber("Speaker Heading", localizer.getSpeakerHeading().getDegrees());
        SmartDashboard.putNumber("Speaker X", localizer.getSpeakerPosition().getX());
        SmartDashboard.putNumber("Speaker Y", localizer.getSpeakerPosition().getY());

        // SmartDashboard.putNumber("global orientation", localizer.getOrientation().getDegrees());
        // SmartDashboard.putNumber("vx", drive.getVelocities().vxMetersPerSecond);
        // SmartDashboard.putNumber("vy", drive.getVelocities().vyMetersPerSecond);
        localizer.step();
        // SmartDashboard.putNumber("startposex", localizer.startingPose2d.getX());
        // SmartDashboard.putNumber("startposey", localizer.startingPose2d.getY());
        var pose2d = localizer.getDisplacementPose2d();
        // SmartDashboard.putNumber("xpose2d", pose2d.getX());
        // SmartDashboard.putNumber("ypose2d", pose2d.getY());
        // SmartDashboard.putNumber("Pigeon Offset", localizer.pigeonOffset.getDegrees());
        // SmartDashboard.putNumber("FR Rot", drive.rotators[0].getSelectedSensorPosition());
        // SmartDashboard.putNumber("FR Enc Pos", drive.encoders[0].getPosition());
        // SmartDashboard.putNumber("FR Enc Abs", drive.encoders[0].getAbsolutePosition());
        SmartDashboard.putNumber("Robot Heading", localizer.getHeading().getDegrees());
    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    @Override
    public void autonomousInit() {
        intake.removeDefaultCommand();
        shooter.removeDefaultCommand();
        indexer.removeDefaultCommand();
        drive.getRotational().removeDefaultCommand();

        new AlignSpeakerCommand(drive.getRotational(), localizer).schedule();
    //     AutoBuilder.configureHolonomic(
    //         localizer::getDisplacementPose2d,
    //         localizer::resetPose2d,
    //         drive::getVelocities,
    //         drive::setVelocities, 
    //         new HolonomicPathFollowerConfig(
    //             new PIDConstants(7.5, 0, 1),
    //             new PIDConstants(0.25, 0, 0), 
    //             drive.getMaxSpeed(), 
    //             12.3743687 * IN , 
    //             new ReplanningConfig()
    //         ),
    //         () -> {
    //             var alliance = DriverStation.getAlliance();
    //             if(alliance.isPresent()){
    //                 return alliance.get() == DriverStation.Alliance.Red;
    //             }
    //             return false;
    //         } , drive
    //     );
    //     localizer.setup();
    //     auto.schedule();
    //     drive.getTranslational().removeDefaultCommand();
    //     drive.getRotational().removeDefaultCommand();
    //     intake.intake();
    //     indexer.intake();
    //     shooter.intake();
    //     shooter.shoot(.5);
        // new SimpleAutoPlanLeft(drive.getTranslational(), drive.getRotational(), shooter, indexer, intake, elevator, localizer).schedule();
    }

    int t = 0;

    @Override
    public void autonomousPeriodic() {
        // drive.getTranslational().setVelocity(new Translation2d(0, -0.5));
        // shooter.setRotation(65);
        // shooter.shoot(0.7);

        // if (t == 200) {
        //     indexer.intake();
        //     shooter.intake();
        // }
        
        // t++;
    }

    @Override
    public void teleopInit() {
        Trigger shootTrigger = new JoystickButton(xbox2, XboxController.Button.kLeftBumper.value);
        shootTrigger.onTrue(new ShooterSpeakerAlgCommand(16, drive.getRotational(), shooter, indexer, localizer, led));
        // for (int i = 0; i < drive.motors.length; i++) {
        //     drive.motors[i].config_kP(0, SmartDashboard.getNumber("swkP", 0.0056));
        //     drive.motors[i].config_kI(0, SmartDashboard.getNumber("swkI", 0.0));
        //     drive.motors[i].config_kD(0, SmartDashboard.getNumber("swkD", 0.06));
        //     drive.motors[i].config_kF(0, SmartDashboard.getNumber("swkF", 0.02));
        // }

        // Main driver
        drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox1, 6));
        drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox1, 2.5 * Math.PI));

        // Second driver
        shooter.setDefaultCommand(new ShooterControlCommand(shooter, xbox2, log));
        elevator.setDefaultCommand(new ElevatorControlCommand(elevator, xbox2,  xbox1));
        intake.setDefaultCommand(new IntakeIndexerControlCommand(intake, indexer, xbox2));
        // Trigger xboxTrigger = new JoystickButton(xbox1, XboxController.Button.kY.value);
        // xboxTrigger.onTrue(new PreSpeakerAlignCommand(drive, localizer, new Rotation2d(0), 0.2 * Math.PI));
    }

    int shooterDir = 1;
    double degrees = 30.0;
    @Override
    public void teleopPeriodic() {
        // for (int i = 0; i < 4; i++) {
        //     drive.rotators[i].config_kP(0, SmartDashboard.getNumber("kP", 0));
        //     drive.rotators[i].config_kI(0, SmartDashboard.getNumber("kI", 0));
        //     drive.rotators[i].config_kD(0, SmartDashboard.getNumber("kD", 0));
        // }


        // drive.OFFSETS[0] = -2930 + 1024 + 2048 + (int) SmartDashboard.getNumber("Delta FL", 0);
        // drive.OFFSETS[2] = -2841 + 1024 + 2048 + (int) SmartDashboard.getNumber("Delta FR", 0);
        // drive.OFFSETS[1] = -3267 + 1024 + 2048 + (int) SmartDashboard.getNumber("Delta BL", 0);
        // drive.OFFSETS[3] = -2143 + 1024 + 2048 + (int) SmartDashboard.getNumber("Delta BR", 0);

        if (localizer.isSpeakerTagVisible()) {
            led.fill(0, 255, 0);
        } else if (indexer.hasNote()) {
            led.fill(255, 20, 0);
        } else {
            led.fill(0, 0, 255);
        }

        // if (xbox1.getXButtonPressed()) {
        //     localizer.resetHeading();
        // }

        // if (xbox2.getLeftBumperPressed()) {
        //     new ShooterSpeakerAlgCommand(8, shooter, indexer, localizer).schedule();
        // }

        // SmartDashboard.putNumber("height", elevator.getEncoder());
        // SmartDashboard.putNumber("linkage angle", shooter.getRotation());
        // if(xbox1.getAButton()){
            // drive.getTranslational().setVelocity(new Translation2d(0, 1));
        // } else drive.getTranslational().setVelocity(new Translation2d(0, 0));
        
        // if(xbox1.getYButton()){
        //     intake.intake();
        //     indexer.intake();
        //     shooter.intake();
        //     shooter.shoot(.2);
        // } else {
        //     intake.stop();
        //     indexer.stop();
        //     shooter.index(0);
        //     shooter.shoot(0);
        // }
        // elevator.leftSpoolMotor.set(ControlMode.PercentOutput, 0.1*(xbox1.getLeftTriggerAxis() - xbox1.getRightTriggerAxis()));
        
        // if(xbox1.getBButton()){
        //     degrees = 15.1 + (64.8 - 15.1) * xbox1.getLeftTriggerAxis();
        // }
        // shooter.setRotation(SmartDashboard.getNumber("D", 0) * Math.PI/180.0);
        // if (xbox1.getLeftBumperPressed()) {
        //     for (int i = 0; i < drive.motors.length; i++) {
        //         SupplyCurrentLimitConfiguration currConfig = new SupplyCurrentLimitConfiguration();
        //         currConfig.currentLimit = 70;
        //         currConfig.enable = true;
        //         currConfig.triggerThresholdCurrent = 80;
        //         currConfig.triggerThresholdTime = 0.01;
        //         drive.motors[i].configSupplyCurrentLimit(currConfig);
                
        //         SupplyCurrentLimitConfiguration rotatorConfig = new SupplyCurrentLimitConfiguration();
        //         rotatorConfig.currentLimit = 40;
        //         rotatorConfig.enable = true;
        //         rotatorConfig.triggerThresholdCurrent = 50;
        //         rotatorConfig.triggerThresholdTime = 0.01;
        //         drive.rotators[i].configSupplyCurrentLimit(currConfig);
        //     }
        // } else if (xbox1.getLeftBumperReleased()) {
        //     for (int i = 0; i < drive.motors.length; i++) {
        //         SupplyCurrentLimitConfiguration currConfig = new SupplyCurrentLimitConfiguration();
        //         currConfig.currentLimit = 30;
        //         currConfig.enable = true;
        //         currConfig.triggerThresholdCurrent = 40;
        //         currConfig.triggerThresholdTime = 0.01;
        //         drive.motors[i].configSupplyCurrentLimit(currConfig);
                
        //         SupplyCurrentLimitConfiguration rotatorConfig = new SupplyCurrentLimitConfiguration();
        //         rotatorConfig.currentLimit = 20;
        //         rotatorConfig.enable = true;
        //         rotatorConfig.triggerThresholdCurrent = 30;
        //         rotatorConfig.triggerThresholdTime = 0.01;
        //         drive.rotators[i].configSupplyCurrentLimit(currConfig);
        //     }
        // }


    }
}