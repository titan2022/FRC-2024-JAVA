// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class TeleIntakeCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public XboxController xbox;
    public IndexerSubsystem indexer;
    public IntakeSubsystem intake;


    public TeleIntakeCommand(IndexerSubsystem indexer, IntakeSubsystem intake, XboxController xbox) {
        this.xbox = xbox;
        this.intake = intake;
        this.indexer = indexer;

        addRequirements(intake, indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (xbox.getYButton()) {
            intake.intake();
            indexer.intake();
        } else {
            intake.stop();
            indexer.stop();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.stop();
        indexer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
