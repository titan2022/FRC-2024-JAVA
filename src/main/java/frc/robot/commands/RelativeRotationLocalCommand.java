// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.utility.Localizer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Creates a rotation command in radians which
 * goes counter clockwise with the front of the
 * robot being 0
 */
public class RelativeRotationLocalCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final RotationalDrivebase rotDriveBase;
  private final Localizer localizer;
  private final double targetAngle;
  private final double speed_radpers; // rad/s
  private final double tolerance;
  private double initialAngle;
  private double curAngle;

  /**
   * Creates a rotation command relative to the front of the robot
   * 
   * @param subsystem Drivebase
   * @param localizer The localizer which tracks the position and orientation of
   *                  the robot
   * @param angle     The angle of rotation in radians counterclockwise relative
   *                  to the front of the robot
   * @param speed     Speed at which to execute the rotation
   */
  public RelativeRotationLocalCommand(RotationalDrivebase subsystem, Localizer localizer, Rotation2d angle,
      double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.rotDriveBase = subsystem;
    this.localizer = localizer;
    this.targetAngle = angle.getRadians();
    this.curAngle = 0;
    this.speed_radpers = speed;
    this.speed = speed / 50;
    this.tolerance = 1; // 1 radian
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = localizer.getLocalOrientation();
    rotDriveBase.setRotation(speed_radpers);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curAngle = localizer.getLocalOrientation() - initialAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotDriveBase.setRotation(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (curAngle >= targetAngle - tolerance && curAngle <= targetAngle + tolerance) {
      return true;
    } else {
      return false;
    }
  }
}
