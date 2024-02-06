// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SlamDunkerSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;

import static frc.robot.utility.Constants.getSwerveDriveTalonDriveConfig;
import static frc.robot.utility.Constants.getSwerveDriveTalonRotaryConfig;

public class Robot extends TimedRobot {
    // private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(getSwerveDriveTalonDriveConfig(), getSwerveDriveTalonRotaryConfig());
	private final XboxController xbox = new XboxController(0);
    // private Localizer localizer = new Localizer();
    private static final SlamDunkerSubsystem slamDunker = new SlamDunkerSubsystem();
    private static final IntakeSubsystem intake = new IntakeSubsystem();
    private static final ShooterSubsystem shooter = new ShooterSubsystem();
    @Override
    public void robotInit() {
        // localizer.setup();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Xbox Right Y", xbox.getRightY());
        SmartDashboard.putNumber("Rotator Absolute Position", slamDunker.getRotation());
        SmartDashboard.putNumber("Rotator Ticks per Rotation", slamDunker.rotationEncoder.getDistancePerRotation());
        SmartDashboard.putNumber("Rotator Distance", slamDunker.rotationEncoder.getDistance());
        // localizer.step();

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
        
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // slamDunker.testRotation(0.1);
        // drive.getTranslational().setVelocity(new Translation2d(0, 0.5));
    }

    @Override
    public void teleopInit() {
        // drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), localizer, xbox, 6));
		// drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), localizer, xbox, 1.5 * Math.PI));

        // if (xbox.getBButton()) {
        //     drive.brake();
        // }
    }

    @Override
    public void teleopPeriodic() {
        // if (xbox.getYButton()) {
        //     slamDunker.testRotation(-0.1);
        // } else if (xbox.getAButton()) {
        //     slamDunker.testRotation(0.1);
        // } else {
        //     slamDunker.testRotation(0);
        // }

        // if (xbox.getYButton()) {
        //     shooter.setRotation(shooter.getRotation().minus(Rotation2d.fromDegrees(1)));
        // } else if (xbox.getAButton()) {
        //     shooter.setRotation(shooter.getRotation().plus(Rotation2d.fromDegrees(1)));
        // } else {
            
        // }

        if (xbox.getYButton()) {
            intake.testRotation(-0.2);
        } else if (xbox.getAButton()) {
            intake.testRotation(0.2);
        } else {
            intake.testRotation(0);
        }
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