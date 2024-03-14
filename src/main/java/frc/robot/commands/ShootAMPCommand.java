// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;


/** An example command that uses an example subsystem. */
public class ShootAMPCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double DURATION = 1;
    public static double TIMER = 0;
    public IndexerSubsystem indexer;
    public double endTime;

    public ShootAMPCommand(IndexerSubsystem indexer) {
        this.indexer = indexer;
        TIMER += 5;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        endTime = Timer.getFPGATimestamp() + DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexer.reverse();
        if (TIMER > 0)
            TIMER--;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
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
