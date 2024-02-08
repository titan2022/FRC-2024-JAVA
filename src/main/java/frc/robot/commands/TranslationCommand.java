// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TranslationalDrivebase;

/** An example command that uses an example subsystem. */
public class TranslationCommand extends Command implements VariantCommand {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  protected boolean onBlueSide = true;
  protected Translation2d velocity;

  public TranslationCommand(double x, double y, double speed, TranslationalDrivebase driveBase) {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean onBlueSide() {
    return onBlueSide;
  }

  @Override
  public void changeColorSide() {
    velocity = new Translation2d(velocity.getX(), -velocity.getY());
    onBlueSide = !onBlueSide;
  }
}
