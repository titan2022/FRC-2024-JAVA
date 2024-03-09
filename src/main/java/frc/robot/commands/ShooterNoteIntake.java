// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;


/** An example command that uses an example subsystem. */
public class ShooterNoteIntake extends Command {
    public static final double SHOOTER_INTAKE_SPEED = -0.5;
    public static final double INDEXER_SPEED = -0.5;

    public ShooterSubsystem shooter;
    public ElevatorSubsystem elevator;

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShooterNoteIntake(ElevatorSubsystem elevator, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.elevator = elevator;

        addRequirements(elevator, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(SHOOTER_INTAKE_SPEED);
        elevator.index(INDEXER_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
        elevator.index(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (elevator.hasNote())
            return true;
        else 
            return false;
    }
}
