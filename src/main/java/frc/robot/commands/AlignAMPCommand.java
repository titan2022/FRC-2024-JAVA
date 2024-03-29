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
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;


/** An example command that uses an example subsystem. */
public class AlignAMPCommand extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public static final int BLUE_AMP_APRILTAG = 6;
  public static final int RED_AMP_APRILTAG = 5;

  public static final double TRANSLATIONAL_SPEED = 1;
  public static final Rotation2d ROTATIONAL_SPEED = new Rotation2d(1);

  public AlignAMPCommand(TranslationalDrivebase translationalDrive, RotationalDrivebase rotationalDrive, Localizer localizer) {
    Translation2d distanceFromAprilTag;
    Rotation2d angleOfAprilTag;
    if (Constants.getColor() == Alliance.Blue) {
      distanceFromAprilTag = localizer.getTagPosition(6);
      angleOfAprilTag = localizer.getTagHeading(6);
    } else {
      distanceFromAprilTag = localizer.getTagPosition(5);
      angleOfAprilTag = localizer.getTagHeading(5);
    }


    addCommands(
      new RotationCommand(angleOfAprilTag, ROTATIONAL_SPEED, rotationalDrive, localizer),
      new TranslationCommand(distanceFromAprilTag, TRANSLATIONAL_SPEED, translationalDrive)
    );
    
    addRequirements(translationalDrive, rotationalDrive);
  }
}
