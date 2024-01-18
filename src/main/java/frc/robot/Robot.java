// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);
    // private Localizer localizer = new Localizer(0, 0, 0);
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(getSwerveDriveTalonDriveConfig(), getSwerveDriveTalonRotaryConfig());
    private final XboxController xbox = new XboxController(0);

    @Override
    public void robotInit() {

    }

    @Override
    public void robotPeriodic() {
        // CommandScheduler.getInstance().run();
        // SmartDashboard.putNumber("Local X", localizer.getLocalPosition().getX());
        // SmartDashboard.putNumber("Local Y", localizer.getLocalPosition().getY());
        // SmartDashboard.putNumber("Local Orientation", new Rotation2d(localizer.getLocalOrientation()).getDegrees());
        // SmartDashboard.putNumber("Global X", localizer.getGlobalPosition().getX());
        // SmartDashboard.putNumber("Global Y", localizer.getGlobalPosition().getY());
        // SmartDashboard.putNumber("Global Orientation", new Rotation2d(localizer.getGlobalOrientation()).getDegrees());
    }

    @Override
    public void disabledInit() {
        drive.brake();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(
            new TranslationCommand(0, 1, 0.25, drive.getTranslational())
        );
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        drive.brake();

        // drive.getTranslational()
        //         .setDefaultCommand(new XBoxTranslationalCommand(drive.getTranslational(), xbox, pigeon, 1));
        // drive.getRotational()
        //         .setDefaultCommand(new XBoxRotationalCommand(drive.getRotational(), xbox, 3 * Math.PI, pigeon));
    }

    @Override
    public void teleopPeriodic() {

    }
}