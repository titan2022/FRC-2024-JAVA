// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utility.Localizer;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Creates a translation command in meters to move
 * relative to the front of the robot
 */
public class RelativeTranslationTimedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TranslationalDrivebase transDriveBase;
  private final Translation2d deltaPosition;
  private final double speed;
  private final double tolerance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RelativeTranslationTimedCommand(TranslationalDrivebase transDriveBase, Translation2d deltaPosition, double speed) {
    this.transDriveBase = transDriveBase;
    this.deltaPosition = deltaPosition;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transDriveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
