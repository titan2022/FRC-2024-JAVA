// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class MoveToNoteCommand extends ParallelDeadlineGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private IntakeSubsystem intake;
  private TranslationalDrivebase driveBase;
  private Localizer localizer;


  public MoveToNoteCommand(IntakeSubsystem intake, TranslationalDrivebase driveBase, Localizer localizer) {
    super(new TranslationCommand(new Translation2d(), 0, driveBase), new NoteIntakeCommand(intake)); 

    addRequirements(intake, driveBase);
  }

}
