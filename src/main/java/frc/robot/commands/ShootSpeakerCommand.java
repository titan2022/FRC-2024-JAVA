// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootSpeakerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double RAMP_TIME = 1;
    public static final double SHOOT_DURATION = 0.25;
    public static final double SHOOTER_INDEX_SPEED = 0.6;
    public static final double ELEVATOR_INDEX_SPEED = -0.6;

    public ShooterSubsystem shooter;
    public IndexerSubsystem indexer; 
    public double speed;
    public double rampTime;
    public double endTime; 
    
    public ShootSpeakerCommand(double speed, ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.speed = speed;
        this.shooter = shooter;
        this.indexer = indexer;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        rampTime = Timer.getFPGATimestamp() + RAMP_TIME;
        endTime = rampTime + SHOOT_DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Timer.getFPGATimestamp() < rampTime){
            shooter.holdAngle();
            shooter.shoot(speed);
        }
        else {
            shooter.holdAngle();
            shooter.shoot(speed);
            shooter.intake();
            indexer.intake();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
        shooter.holdIndex();
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

