// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.Constants;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.Unit;

/** An example command that uses an example subsystem. */
public class ShooterSpeakerAlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;
    public static double SPEED = 0.6;
    public static final double SPEAKER_HEIGHT = 2 * Unit.METERS;
    public static final double SUBWOOFER_LENGTH = 0.91;
    public ShooterSubsystem shooter;
    public Localizer localizer;
    public Rotation2d shootAngle;
    public Rotation2d rotateAngle;
    public boolean nearTarget = false;

    public ShooterSpeakerAlignCommand(ShooterSubsystem shooter, Localizer localizer) {
        this.shooter = shooter;
        this.localizer = localizer;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d aprilTagDistance;
        if (Constants.getColor() == Alliance.Blue) {
            aprilTagDistance = localizer.getTagPosition(BLUE_SPEAKER_APRILTAG);
        } else {
            aprilTagDistance = localizer.getTagPosition(RED_SPEAKER_APRILTAG);
        }

        double x = aprilTagDistance.getNorm();
        Translation2d vectorToSpeaker = new Translation2d(x, SPEAKER_HEIGHT);
    
        Rotation2d shootAngle = vectorToSpeaker.getAngle();
        
        // rotational.setRotationalVelocity(aprilTagRotation);
        // shooter.setRotation(shootAngle);
        // shooter.shoot(SPEED);
        
       nearTarget = shooter.setRotation(shootAngle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setRotation(shootAngle);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (nearTarget) 
            return true;
        else 
            return false;
    }
}
