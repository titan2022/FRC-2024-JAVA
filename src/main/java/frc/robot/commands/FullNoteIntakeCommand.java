// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullNoteIntakeCommand extends SequentialCommandGroup {
    public static final double MOVE_OVERSHOOT = 0.5;
    public static final double OVER_SHOOT_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullNoteIntakeCommand(TranslationalDrivebase translational, RotationalDrivebase rotational, IntakeSubsystem intake, ElevatorSubsystem elevator, Localizer localizer) {
        addCommands(
            new ParallelCommandGroup(
                new AlignNoteCommand(translational, rotational, intake, localizer),
                new MoveElevatorCommand(false, elevator)
            ),
            new ParallelDeadlineGroup(
                new NoteIntakeCommand(elevator, intake),
                new TranslationCommand(new Translation2d(0, MOVE_OVERSHOOT), OVER_SHOOT_SPEED, translational)
            )   
        );

        addRequirements(translational, rotational, intake, elevator);
    }

}
