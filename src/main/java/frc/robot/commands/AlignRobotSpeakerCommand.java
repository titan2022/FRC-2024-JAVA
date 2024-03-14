// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Constants;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Localizer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignRobotSpeakerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double SHOOTER_INDEX_SPEED = 0.6;
    // public static final double ELEVATOR_INDEX_SPEED = -0.6;
    // public double speed;
    // public double rampTime;
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;
    public static double ANGLE_DEADBAND = 2 * Unit.DEG;
    public static double SPEED = 0.6;
    public static final double SPEAKER_HEIGHT = 2 * Unit.METERS;
    public static final double SUBWOOFER_LENGTH = 0.91;
    public RotationalDrivebase rotational;
    public Localizer localizer;
    public Rotation2d deltaAngle;
    
    public AlignRobotSpeakerCommand(RotationalDrivebase rotational, Localizer localizer) {
        // this.speed = speed;
        this.rotational = rotational;
        this.localizer = localizer;

        addRequirements(rotational);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Constants.getColor() == Alliance.Blue) {
            deltaAngle = localizer.getTagRotation(BLUE_SPEAKER_APRILTAG);
        } else {
            deltaAngle = localizer.getTagHeading(RED_SPEAKER_APRILTAG);
        }

        rotational.setRotationalVelocity(deltaAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        rotational.setRotationalVelocity(new Rotation2d(0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(deltaAngle.getRadians()) < ANGLE_DEADBAND)
            return true;
        else
            return false;

    }
}

