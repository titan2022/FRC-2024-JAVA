// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class MoveElevatorCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAISE_SPEED = 0.1;
    // public static final double LOWER_SPEED = -0.1;
    public static double HIGH_SPEED = 1;
    public static double LOW_SPEED = 0.2;

    public static double HIGH_SPEED_TIME = 0.6;

    public ElevatorSubsystem elevator;
    public boolean upward;
    public double highSpeedTime;
    
    public MoveElevatorCommand(boolean up, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.upward = up;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // SmartDashboard.putBoolean("MoveElevator", true);
        highSpeedTime = Timer.getFPGATimestamp() + HIGH_SPEED_TIME;
        SmartDashboard.putNumber("Start Time", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("End Time", highSpeedTime);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = 1;
        //Sets sign
        if (upward)
            speed *= 1;
        else 
            speed *= -1;

        if (Timer.getFPGATimestamp() < highSpeedTime)
            speed *= HIGH_SPEED;
        else 
            speed *= LOW_SPEED;

        SmartDashboard.putNumber("Elevator Speed", speed);

        elevator.elevate(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.hold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (elevator.isStalling() && Timer.getFPGATimestamp() > highSpeedTime) {
            SmartDashboard.putBoolean("Finished Command", true);
            return true;
        }
        else {
            SmartDashboard.putBoolean("Finished Command", false);
            return false;
        }
    }
  }
