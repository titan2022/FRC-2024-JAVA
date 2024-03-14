// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class TeleElevatorCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAISE_SPEED = 0.1;
    // public static final double LOWER_SPEED = -0.1;
    public static final double DEADZONE = 0.5;
    public XboxController xbox;
    public ElevatorSubsystem elevator;
    public double highSpeedTime;
    // public boolean passCheck = false;
    
    public TeleElevatorCommand(ElevatorSubsystem elevator, XboxController xbox) {
        this.elevator = elevator;
        this.xbox = xbox;
        
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("TeleEle", SmartDashboard.getNumber("TeleEle", 0) + 1);
        if (xbox.getLeftY() > DEADZONE)
            elevator.raise();
        else if (xbox.getLeftY() < -DEADZONE)
            elevator.lower();
        else if (xbox.getXButton())
            // elevator.elevate(-SmartDashboard.getNumber("A", highSpeedTime) * xbox.getRightTriggerAxis());
            elevator.elevate(-SmartDashboard.getNumber("A", highSpeedTime) * xbox.getRightTriggerAxis());
        else 
            elevator.hold();
        // double speed = 1;
        // //Sets sign
        // if (up)
        //     speed *= 1;
        // else 
        //     speed *= -1;

        // // if (Timer.getFPGATimestamp() < highSpeedTime)
        // //     speed *= HIGH_SPEED;
        // // else 
        // //     speed *= LOW_SPEED;

        // speed *= LOW_SPEED;

        // elevator.elevate(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.hold();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
  }
