// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

import static frc.robot.utility.Constants.getSwerveDriveTalonDriveConfig;
import static frc.robot.utility.Constants.getSwerveDriveTalonRotaryConfig;

public class Robot extends TimedRobot {
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(getSwerveDriveTalonDriveConfig(), getSwerveDriveTalonRotaryConfig());
	private final XboxController xbox = new XboxController(0);
    private Localizer localizer = new Localizer();

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        localizer.step();
        
        SmartDashboard.putNumber("Global X", localizer.getPosition().getX());
        SmartDashboard.putNumber("Global Y", localizer.getPosition().getY());
        SmartDashboard.putNumber("Global Orientation", localizer.getOrientation().getDegrees());
        SmartDashboard.putNumber("Global Heading", localizer.getHeading().getDegrees());
    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    @Override
    public void autonomousInit() {
        
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), xbox, 0.8));
		drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), xbox, 0.8 * Math.PI));
    }

    @Override
    public void teleopPeriodic() {

    }
}