// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utility.Localizer;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Creates a translation command in meters to move
 * relative to the front of the robot
 */
public class RelativeTranslationLocalCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TranslationalDrivebase transDriveBase;
  private final Localizer localizer;
  private final Translation2d deltaPosition;
  private final double speed;
  private final double tolerance = 0.1;
  private Translation2d finalPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RelativeTranslationLocalCommand(TranslationalDrivebase transDriveBase, Localizer localizer, Translation2d deltaPosition, double speed) {
    this.transDriveBase = transDriveBase;
    this.localizer = localizer;
    this.deltaPosition = deltaPosition;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(transDriveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d displacementVector = deltaPosition.rotateBy(new Rotation2d(localizer.getLocalOrientation()));
    finalPosition = localizer.getLocalPosition().plus(displacementVector);
    Translation2d normalizedVector = deltaPosition.div(deltaPosition.getNorm());
    normalizedVector.times(speed);
    transDriveBase.setVelocity(normalizedVector);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transDriveBase.setVelocity(new Translation2d(0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (localizer.getLocalPosition().getDistance(finalPosition) < tolerance) {
      return true;
    } else {
      return false;
    }
  }
}
