// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.RobotSize;

/** An example command that uses an example subsystem. */
public class AlignNoteCommand extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public static final double TRANSLATIONAL_SPEED = 1;
  public static final Rotation2d ROTATIONAL_SPEED = new Rotation2d(1);
  public static final double forwardWalk = 0.25;

  public AlignNoteCommand(TranslationalDrivebase translationalDrivebase, RotationalDrivebase rotationalDrive, Localizer localizer) {
    // super(new TranslationCommand(new Translation2d(), speed, driveBase), new NoteIntakeCommand(intake)); 
    Translation2d movement = localizer.getNotePosition();
    movement.minus(new Translation2d(0, (RobotSize.LENGTH / 2) - forwardWalk));

    addCommands(new TranslationCommand(movement, TRANSLATIONAL_SPEED, translationalDrivebase));

    addRequirements(translationalDrivebase, rotationalDrive);
  }
}
