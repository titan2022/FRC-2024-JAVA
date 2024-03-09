// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootSpeakerCommand extends Command {
    public static final double RAMP_TIME = 1;
    public static final double SHOOT_DURATION = 0.25;
    public static final double INDEX_SPEED = 0.5;


    public ShooterSubsystem shooter;
    public ElevatorSubsystem elevator;
    public double speed;
    public double rampTime;
    public double endTime; 
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShootSpeakerCommand(double speed, ShooterSubsystem shooter, ElevatorSubsystem elevator) {
        this.speed = speed;
        this.shooter = shooter;
        this.elevator = elevator;

        addRequirements(shooter, elevator);
    }

    @Override
    public void initialize() {
        rampTime = Timer.getFPGATimestamp() + RAMP_TIME;
        endTime = rampTime + SHOOT_DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() < rampTime)
            shooter.shoot(speed);
        else {
            shooter.shoot(speed);
            shooter.index(INDEX_SPEED);
            elevator.index(INDEX_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
        shooter.index(0);
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

