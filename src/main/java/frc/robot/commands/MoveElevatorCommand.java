// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class MoveElevatorCommand extends Command {
    public static final double RAISE_SPEED = 0.5;
    public static final double LOWER_SPEED = -0.5;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ElevatorSubsystem elevator;
    public boolean upward;
    public MoveElevatorCommand(boolean up, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.upward = up;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (upward)
            elevator.raise(RAISE_SPEED);
        else 
            elevator.lower(LOWER_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.hold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (elevator.isStalling())
            return true;
        else 
            return false;
    }
  }
