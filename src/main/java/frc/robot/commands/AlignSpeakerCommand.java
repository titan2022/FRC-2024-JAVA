// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class AlignSpeakerCommand extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public static final double translationalSpeed = 1;
  public static final Rotation2d rotationalSeed = new Rotation2d(1);

  public AlignSpeakerCommand(TranslationalDrivebase translationalDrive, RotationalDrivebase rotationalDrive, Localizer localizer) {
    Translation2d distanceFromAprilTag = new Translation2d(0, 0);
    Rotation2d angleOfAprilTag = new Rotation2d(0);

    
    addRequirements(translationalDrive, rotationalDrive);
  }
}
