// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.utility.Constants.getSwerveDriveTalonDriveConfig;
import static frc.robot.utility.Constants.getSwerveDriveTalonRotaryConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RotationalDriveCommand;
import frc.robot.commands.TranslationCommand;
import frc.robot.commands.TranslationalDriveCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

public class Robot extends TimedRobot {
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(getSwerveDriveTalonDriveConfig(), getSwerveDriveTalonRotaryConfig());
	private final XboxController xbox = new XboxController(0);
    private IntakeSubsystem intake = new IntakeSubsystem();
    private Localizer localizer = new Localizer();
    
    @Override
    public void robotInit() {
        SmartDashboard.putNumber("Desired Intake Speed", 0.75);
        SmartDashboard.putNumber("Desired Speed", 0.5);
        SmartDashboard.putNumber("Desired X Position", 0);
        SmartDashboard.putNumber("Desired Y Position", 0);
        SmartDashboard.putNumber("Desired Rotation", 0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Current Angle", localizer.getHeading().getDegrees());
        localizer.step();



        // SmartDashboard.putNumber("Rotation", shooter.getRotation().getDegrees());
        
        // SmartDashboard.putNumber("Global X", localizer.getPosition().getX());
        // SmartDashboard.putNumber("Global Y", localizer.getPosition().getY());
        // SmartDashboard.putNumber("Global Orientation", localizer.getOrientation().getDegrees());
        // SmartDashboard.putNumber("Global Heading", localizer.getHeading().getDegrees());
    }

    @Override
    public void disabledInit() {
        // drive.brake();
    }

    @Override
    public void autonomousInit() {
        localizer.setup();
        CommandScheduler.getInstance().schedule(
            new TranslationCommand(new Translation2d(SmartDashboard.getNumber("Desired X Position", 0), SmartDashboard.getNumber("Desired Y Position", 0)), SmartDashboard.getNumber("Desired Speed", 0), drive.getTranslational())
        );
        // drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox, 1));
		// drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox, Math.PI / 2));
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        

    }

    @Override
    public void teleopInit() {
        localizer.setup();
        drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox, 1));
		drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox, Math.PI / 2));
    }

    @Override
    public void teleopPeriodic() {
        // drive.getTranslational().setVelocity(new Translation2d(0, 0.5));

        // if (xbox.getYButtonPressed()) {
        //     intake.setWheelSpeed(SmartDashboard.getNumber("Desired Intake Speed", 0));
        // } else if (xbox.getAButtonPressed())
        //     intake.stop();

        // if (xbox.getYButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(0, SmartDashboard.getNumber("Desired Y Velocity", 0)));
        // } else if (xbox.getAButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(0, -1 * SmartDashboard.getNumber("Desired Y Velocity", 0)));
        // } else if (xbox.getXButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(SmartDashboard.getNumber("Desired X Velocity", 0), 0));
        // } else if(xbox.getBButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(-1 * SmartDashboard.getNumber("Desired X Velocity", 0), 0));
        // } else {
        //     drive.getTranslational().setVelocity(new Translation2d(0, 0));
        // }

        // SmartDashboard.putBoolean("XButton", xbox.getXButton());
        // if (xbox.getXButton()) {
        //     intake.testWheelMotor(0.2);
        // } else if (xbox.getBButton()) {
        //     intake.testWheelMotor(-0.2);
        // } else if (xbox.getYButton()) {
        //     slamDunker.testRotation(0.1);
        // } else if (xbox.getAButton()) {
        //     slamDunker.testRotation(-0.1);
        // } else if (xbox.getRightBumper()) {
        //     slamDunker.testWheelRotation(0.5);
        // } else if (xbox.getLeftBumper()) {
        //     slamDunker.testWheelRotation(-0.5);
        // }

        // slamDunker.testWheelRotation(0.2);
        // intake.testWheelMotor(0.2);
    }
}