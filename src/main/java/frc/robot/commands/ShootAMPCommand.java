// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class ShootAMPCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double DURATION = 0.25;
    public ElevatorSubsystem elevator;
    public double speed;
    public double endTime;

    public ShootAMPCommand(double speed, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.speed = speed;

        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.index(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.index(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() > endTime) 
            return true;
        else 
            return false;
    }
  }
