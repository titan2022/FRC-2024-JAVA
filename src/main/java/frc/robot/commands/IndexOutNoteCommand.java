// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SlamDunkerSubsystem;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class IndexOutNoteCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static double DURATION_AFTER_NOTE = 0.25;
    public double speed;
    public ElevatorSubsystem elevator;
    public double endTime;
    public boolean endCommand;

    public IndexOutNoteCommand(double speed, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.speed = speed;

        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.indexSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.indexSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!elevator.hasNote() && !endCommand){
            endTime = Timer.getFPGATimestamp();
            endCommand = true;
        }
        
        if (endCommand && Timer.getFPGATimestamp() > endTime) 
            return true;
        else 
            return false;
    }
  }
