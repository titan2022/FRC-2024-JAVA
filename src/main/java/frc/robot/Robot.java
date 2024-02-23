// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utility.Localizer;
import static frc.robot.utility.Constants.Unit.*;

import static frc.robot.utility.Constants.getSwerveDriveTalonDriveConfig;
import static frc.robot.utility.Constants.getSwerveDriveTalonRotaryConfig;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Robot extends TimedRobot {
    private SwerveDriveSubsystem drive = new SwerveDriveSubsystem(getSwerveDriveTalonDriveConfig(), getSwerveDriveTalonRotaryConfig());
	private final XboxController xbox = new XboxController(0);
    private IntakeSubsystem intake = new IntakeSubsystem();
    // private Localizer localizer = new Localizer();
    // private static final SlamDunkerSubsystem slamDunker = new SlamDunkerSubsystem();
    // private static final IntakeSubsystem intake = new IntakeSubsystem();
    // WPI_TalonFX motorLeft = new WPI_TalonFX(19);
    // WPI_TalonFX motorRight = new WPI_TalonFX(21);
    @Override
    public void robotInit() {
        SmartDashboard.putNumber("Intake Speed", 0.75);
        // motorLeft.follow(motorRight);
        // motorLeft.setInverted(true);
        // SmartDashboard.putNumber("Rotations Per Sec", 0);
        SmartDashboard.putNumber("X Position", 0);
        SmartDashboard.putNumber("Y Position", 0);
        SmartDashboard.putNumber("Rotation", 0);

        // SmartDashboard.putNumber("Desired X Velocity", 0);
        // SmartDashboard.putNumber("Desired Y Velocity", 0.1);
        

    }

    @Override
    public void robotPeriodic() {
        // CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        SmartDashboard.putNumber("Current X Velocity", drive.getTranslational().getVelocity().getX());
        SmartDashboard.putNumber("Current Y Velocity", drive.getTranslational().getVelocity().getY());
        // SmartDashboard.putNumber("Xbox Right Y", xbox.getRightY());
        // SmartDashboard.putNumber("Rotator Absolute Position", slamDunker.getRotation());
        // SmartDashboard.putNumber("Rotator Ticks per Rotation", slamDunker.rotationEncoder.getDistancePerRotation());
        // SmartDashboard.putNumber("Rotator Distance", slamDunker.rotationEncoder.getDistance());
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
        // intake.setWheelSpeed(SmartDashboard.getNumber("Intake Speed", 0));
        CommandScheduler.getInstance().schedule(new TranslationCommand(new Translation2d(SmartDashboard.getNumber("X Position", 0), SmartDashboard.getNumber("Y Position", 0)), 0.5, drive.getTranslational()));
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // drive.getTranslational().setVelocity(new Translation2d(0, 0.5));
        if (xbox.getYButton())
            intake.setWheelSpeed(SmartDashboard.getNumber("Intake Speed", 0));
        else if (xbox.getAButton())
            intake.stop();

    }

    @Override
    public void teleopInit() {
        // drive.getTranslational().setDefaultCommand(new TranslationalDriveCommand(drive.getTranslational(), xbox, localizer, 6));
		// drive.getRotational().setDefaultCommand(new RotationalDriveCommand(drive.getRotational(), xbox, 1.5 * Math.PI, localizer));
    }

    @Override
    public void teleopPeriodic() {
        if (xbox.getYButton())
            intake.setWheelSpeed(SmartDashboard.getNumber("Intake Speed", 0));
        else if (xbox.getAButton())
            intake.stop();
        // if (xbox.getYButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(0, SmartDashboard.getNumber("Desired Y Velocity", 0)));
        // } else if (xbox.getAButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(0, -1 * SmartDashboard.getNumber("Desired Y Velocity", 0)));
        // } else if (xbox.getXButton()) {
        //     drive.getTranslational().setVelocity(new Translation2d(SmartDashboard.getNumber("Desired X Velocity", 0), 0));
        // } else if (xbox.getBButton()) {
        //     drive.setRotatorTest();
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