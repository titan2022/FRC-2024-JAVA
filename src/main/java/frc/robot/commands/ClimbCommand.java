// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAISE_SPEED = 0.1;
    // public static final double LOWER_SPEED = -0.1;
    public static final double POWER = -0.5;
    public static final double STALL_LIMIT = 30;

    public ElevatorSubsystem elevator;
    
    public ClimbCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("MoveElevator", true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.elevate(POWER);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        MoveElevatorCommand.up = false;
        elevator.hold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (elevator.isStalling(STALL_LIMIT))
            return true;
        else
            return false;
    }
  }
