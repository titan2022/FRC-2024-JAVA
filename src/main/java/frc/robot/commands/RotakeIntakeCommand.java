// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Backup incase set velocity command does not work properly -i
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class RotateIntakeCommand extends Command {

  double tolerance;
  IntakeSubsystem intake;
  Rotation2D angle;
  Rotation2D velocity; 

  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  public RotateNoteCommand(IntakeSubsystem intake, Rotation2D angle, Rotation2D velocity) {
    addRequirements(intake);
    this.intake = intake;
    this.angle = angle;
    this.tolerance = tolerance;
    this.velocity = velocity;it 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeVelocity(velocity)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    Rotation2D diff = math.absolute(intake.getRotation() - angle);
  
    if(diff < tolerance) {
      return true;
    }

    return false;
  }
}
