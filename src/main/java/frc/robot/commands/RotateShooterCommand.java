// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RotateShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public static final double ANGLE_DEADBAND = 2 * Unit.DEG;
    public ShooterSubsystem shooter;
    public Rotation2d angle;
    public boolean reachedAngle = false;
    
    public RotateShooterCommand(Rotation2d angle, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.angle = angle;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Methods return if reached desired location
        reachedAngle = shooter.setRotation(angle);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.holdAngle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachedAngle;
    }
}

