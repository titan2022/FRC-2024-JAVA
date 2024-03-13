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
    public static double HIGH_SPEED = 0.4;
    public static double LOW_SPEED = 0.2;

    public static double HIGH_SPEED_TIME = 1;

    public ElevatorSubsystem elevator;
    public double highSpeedTime;
    public boolean up;
    // public boolean passCheck = false;
    
    public MoveElevatorCommand(boolean up, ElevatorSubsystem elevator) {
        this.elevator = elevator;
        this.up = up;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MoveElevator", true);
        highSpeedTime = Timer.getFPGATimestamp() + HIGH_SPEED_TIME;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = 1;
        //Sets sign
        if (up)
            speed *= 1;
        else 
            speed *= -1;

        // if (Timer.getFPGATimestamp() < highSpeedTime)
        //     speed *= HIGH_SPEED;
        // else 
        //     speed *= LOW_SPEED;

        speed *= LOW_SPEED;

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
        if (elevator.canRun()) {
            if (up && elevator.getEncoder() < ElevatorSubsystem.TOP_ENCODER_VALUE) 
                return false;
            else if (!up && elevator.getEncoder() > ElevatorSubsystem.BOT_ENCODER_VALUE)
                return false;
            else 
                return true;
        } else 
            return true;
        // if (up) {
        //     if (elevator.getEncoder() > ElevatorSubsystem.TOP_ENCODER_VALUE - 1000)
        //         return true;
        //     else 
        //         return false;
        // } else {
        //     if (elevator.getEncoder() < ElevatorSubsystem.BOT_ENCODER_VALUE + 1000)
        //         return true;
        //     else 
        //         return false;
        // }

    }
  }
