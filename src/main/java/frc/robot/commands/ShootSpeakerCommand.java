// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootSpeakerCommand extends Command {
    public static final double RAMP_UP_TIME = 1;
    public static final double SHOOT_TIME = 0.5;
    public static final double INDEXER_SPEED = 0.5;

    private ShooterSubsystem shooter;
    private double speed;
    private double rampTime;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShootSpeakerCommand(double speed, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.speed = speed;

        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        rampTime = Timer.getFPGATimestamp() + RAMP_UP_TIME;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(speed);
        if (Timer.getFPGATimestamp() > rampTime) {
            shooter.setIndexer(INDEXER_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
        shooter.setIndexer(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() < rampTime + SHOOT_TIME)
            return false;
        else
            return true;
    }
}
