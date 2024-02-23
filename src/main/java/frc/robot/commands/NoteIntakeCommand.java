// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class NoteIntakeCommand extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private IntakeSubsystem intake;
  private TranslationalDrivebase driveBase;
  private Localizer localizer;


  public NoteIntakeCommand(IntakeSubsystem intake, TranslationalDrivebase driveBase, Localizer localizer) {
    this.intake = intake;
    this.driveBase = driveBase;
    this.localizer = localizer;

    addRequirements(driveBase, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
