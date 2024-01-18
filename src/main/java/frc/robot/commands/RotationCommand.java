// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationalDrivebase;

/** An example command that uses an example subsystem. */
public class RotationCommand extends Command {
  private RotationalDrivebase driveBase;
  private Rotation2d angle;
  private double speed;
  private double time;
  private double startTime;
  private double endTime;

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  public RotationCommand(double angle, double speed, RotationalDrivebase driveBase) {
    this.driveBase = driveBase;
    this.angle = Rotation2d.fromDegrees(angle);
    this.speed = speed;

    time = this.angle.getDegrees() / speed;

    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    endTime = startTime + time;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveBase.setRotation(angle.getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setRotation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() < endTime) {
      return false;
    } else {
      return true;
    }
  }
}
