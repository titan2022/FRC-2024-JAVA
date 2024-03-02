// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlamDunkerSubsystem;;

/** An example command that uses an example subsystem. */
public class MoveElevatorCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final DEAD_BAND = 0.1;
    public ElevatorSubsystem elevator;
    public double height;
    public MoveElevatorCommand(double height, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.height = height;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setHeight(height);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(elevator.getHeight() - height) < 0.1)
            return true;
        else 
            return false;
    }
  }
