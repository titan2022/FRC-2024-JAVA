// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Utility;

public class TranslationCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  public static final double velocityDeadband = 0.005;
  public static final double kP = 1.5;
  public static final double frictionOffset = 0.5;
  private TranslationalDrivebase driveBase;
  private Translation2d velocity;
  private double time;
  private double startTime;
  private double endTime;

  public TranslationCommand(double x, double y, double speed, TranslationalDrivebase driveBase) {
    this.driveBase = driveBase;
    Translation2d normalVector = new Translation2d(x, y);
    double distance = normalVector.getNorm();
    velocity = Utility.scaleMagnitude(normalVector, speed);
    time = distance / speed;

    SmartDashboard.putNumber("Command X velocity", velocity.getX());
    SmartDashboard.putNumber("Command Y velocity", velocity.getY());
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
    Translation2d controlVelocity;
    // SmartDashboard.putNumber("StartTime", startTime);
    // SmartDashboard.putNumber("EndTime", endTime);
    // SmartDashboard.putNumber("CurrentTime", Timer.getFPGATimestamp());
    // driveBase.setVelocity(velocity);
    Translation2d error = velocity.minus(driveBase.getVelocity());
    Translation2d frictionCompenstation = Utility.scaleMagnitude(error, frictionOffset);
    Translation2d proportionalTerm = error.times(kP);

    // if (error.getNorm() < velocityDeadband) {
    //   controlVelocity = new Translation2d(0, 0);
    // } else {
    //   controlVelocity = error.times(kP);
    // }
    controlVelocity = proportionalTerm.plus(frictionCompenstation);

    SmartDashboard.putNumber("Control Velocity X", controlVelocity.getX());
    SmartDashboard.putNumber("Control Velocity Y", controlVelocity.getY());
    driveBase.addVelocity(controlVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Reached end() 1", true);
    driveBase.setVelocity(new Translation2d(0, 0));
    // SmartDashboard.putBoolean("Reached end() 2", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() < endTime) {
      // SmartDashboard.putBoolean("isFinished", false);
      return false;
    } else {
      // SmartDashboard.putBoolean("isFinished", false);
      return true;
    }
  }
}
