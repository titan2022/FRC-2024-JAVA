// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.Constants;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class AlignSpeakerCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;

    public static final double TRANSLATIONAL_SPEED = 1;
    public static final Rotation2d ROTATIONAL_SPEED = new Rotation2d(1);

    public static final double SUBWOOFER_LENGTH = 0.91;

    public AlignSpeakerCommand(TranslationalDrivebase translationalDrive, RotationalDrivebase rotationalDrive, Localizer localizer) {
        Translation2d distanceFromAprilTag;
        Rotation2d angleOfAprilTag;

        if (Constants.getColor() == Alliance.Blue) {
            distanceFromAprilTag = localizer.getTagPosition(BLUE_SPEAKER_APRILTAG);
            angleOfAprilTag = localizer.getTagHeading(BLUE_SPEAKER_APRILTAG);
        } else {
            distanceFromAprilTag = localizer.getTagPosition(RED_SPEAKER_APRILTAG);
            angleOfAprilTag = localizer.getTagHeading(RED_SPEAKER_APRILTAG);
        }

        distanceFromAprilTag.minus(new Translation2d(0, SUBWOOFER_LENGTH));

    addCommands(
      new RotationCommand(angleOfAprilTag, ROTATIONAL_SPEED, rotationalDrive, localizer),
      new TranslationCommand(distanceFromAprilTag, TRANSLATIONAL_SPEED, translationalDrive)
    );
    
    addRequirements(translationalDrive, rotationalDrive);
  }
}
