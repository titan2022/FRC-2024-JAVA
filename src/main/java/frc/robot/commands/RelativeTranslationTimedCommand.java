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
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final TranslationalDrivebase drivebase;
  private final Translation2d movement;
  private final double total_time;
  private double cur_time;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param movement  The movement in meters
   * @param speed     The speed in meters per second
   */
  public RelativeTranslationTimedCommand(TranslationalDrivebase drivebase, Translation2d movement,
      double speed) {
    this.drivebase = drivebase;
    this.movement = movement;
    double net_movement = movement.getNorm();
    this.total_time = net_movement / speed;
    this.cur_time = 0;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setVelocity(movement.div(total_time));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 20 ms
    cur_time += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.setVelocity(new Translation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (cur_time >= total_time) {
      return true;
    }
    return false;
  }
}
