// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationalDrivebase;

/** An example command that uses an example subsystem. */
public class RotationCommand extends Command implements VariantCommand{
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  protected boolean onBlueSide = true;
  protected Rotation2d angularVelocity;

  public RotationCommand(double angle, double speed, RotationalDrivebase driveBase) {
    
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
    angularVelocity = new Rotation2d(-angularVelocity.getRadians());
    onBlueSide = !onBlueSide;
  }
}
