package frc.robot;

import static frc.robot.utility.Constants.Unit.IN;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.SimpleAutoPlanLeft;
import frc.robot.commands.control.ElevatorControlCommand;
import frc.robot.commands.control.IntakeIndexerControlCommand;
import frc.robot.commands.control.NoteIntakeCommand;
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
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem();
    private Localizer localizer = new Localizer(drive, false, 5804); 
    private ElevatorSubsystem elevator = new ElevatorSubsystem();
    private IntakeSubsystem intake = new IntakeSubsystem();
    private ShooterSubsystem shooter = new ShooterSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem();
    private DataLog log;
    // private Command auto;

    @Override
    public void robotInit() {
        SmartDashboard.putNumber("Target X", 0);
        SmartDashboard.putNumber("Target Y", 0);
        SmartDashboard.putNumber("Target Omega", 0);
        SmartDashboard.putNumber("kP", 0.1);
        SmartDashboard.putNumber("Tar Shoot Speed", 0);
    //     AutoBuilder.configureHolonomic(
    //         localizer::getDisplacementPose2d,
    //         localizer::resetPose2d,
    //         drive::getVelocities,
    //         drive::setVelocities, 
    //         new HolonomicPathFollowerConfig(
    //             new PIDConstants(7.5, 0, 1),
    //             new PIDConstants(.5, 0, 0), 
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

        DataLogManager.start();
        log = DataLogManager.getLog();
        // auto = new PathPlannerAuto("Test");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Shooter Speed", shooter.getShooterVelocity());

        // Translation2d rotatedVelocity = drive.getTranslational().getVelocity().rotateBy(localizer.getHeading());
        // SmartDashboard.putNumber("Rotated vx", rotatedVelocity.getX());
        // SmartDashboard.putNumber("Rotated vy", rotatedVelocity.getY());
        SmartDashboard.putNumber("Speed", drive.getTranslational().getVelocity().getNorm());
        // SmartDashboard.putNumber("Heading", localizer.getHeading().getDegrees());
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
        // SmartDashboard.putNumber("FL Rot", drive.getRotatorCount(0));
        // SmartDashboard.putNumber("FR Rot", drive.getRotatorCount(1));
        // SmartDashboard.putNumber("BL Rot", drive.getRotatorCount(2));
        // SmartDashboard.putNumber("BR Rot", drive.getRotatorCount(3));
        SmartDashboard.putBoolean("hasNote", indexer.hasNote());
        SmartDashboard.putNumber("FL Enc", drive.getRawRotatorCount(0));
        SmartDashboard.putNumber("FR Enc", drive.getRawRotatorCount(1));
        SmartDashboard.putNumber("BL Enc", drive.getRawRotatorCount(2));
        SmartDashboard.putNumber("BR Enc", drive.getRawRotatorCount(3));

    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
        new NoteIntakeCommand(indexer, intake, shooter).schedule();
        // auto.schedule();

        // new SimpleAutoPlanLeft(drive.getTranslational(), drive.getRotational(), shooter, indexer, intake, elevator, localizer).schedule();
    }

    int t = 0;

    @Override
    public void autonomousPeriodic() {
        // for (int i = 0; i < 4; i++) {
        //     drive.rotators[i].config_kP(0, SmartDashboard.getNumber("kP", 0));
        // }

        drive.getTranslational().setVelocity(new Translation2d(
            SmartDashboard.getNumber("Target X", 0),
            SmartDashboard.getNumber("Target Y", 0)
        )
        );

        drive.getRotational().setRotationalVelocity(
            Rotation2d.fromDegrees(SmartDashboard.getNumber("Target Omega", 0))
        );
    
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
        for (int i = 0; i < 4; i++) {
            drive.rotators[i].config_kP(0, SmartDashboard.getNumber("kP", 0));
        }
        // SmartDashboard.putNumber("height", elevator.getEncoder());
        // SmartDashboard.putNumber("linkage angle", shooter.getRotation());
        // if(xbox1.getAButton()){
            // drive.getTranslational().setVelocity(new Translation2d(0, 1));
        // } else drive.getTranslational().setVelocity(new Translation2d(0, 0));

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

        

        // if (xbox1.getStartButtonPressed()) {
        //     if (!elevator.unlocked) {
        //        shooter.linkageMotor.disable();
        //     }
        // }
    }
}