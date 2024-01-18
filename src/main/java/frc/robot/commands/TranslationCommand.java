// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TranslationalDrivebase;

/** An example command that uses an example subsystem. */
public class TranslationCommand extends Command {
  private TranslationalDrivebase driveBase;
  private Translation2d velocity;
  private double time;
  private double startTime;
  private double endTime;

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  public TranslationCommand(double x, double y, double speed, TranslationalDrivebase driveBase) {
    this.driveBase = driveBase;
    Translation2d normalVector = new Translation2d(x, y);
    double distance = normalVector.getNorm();
    normalVector.div(normalVector.getNorm());
    velocity = normalVector.times(speed);
    time = distance / speed;

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
    driveBase.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.setVelocity(new Translation2d(0, 0));
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
