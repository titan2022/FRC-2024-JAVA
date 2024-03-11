// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullClimbCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double MOVE_SPEED = 1;

    public TranslationalDrivebase translational;
    public RotationalDrivebase rotational;
    public ElevatorSubsystem elevator;
    public Localizer localizer;

    public FullClimbCommand(TranslationalDrivebase translational, RotationalDrivebase rotational, ElevatorSubsystem elevator, Localizer localizer) {
        this.translational = translational;
        this.rotational = rotational;
        this.elevator = elevator;
        this.localizer = localizer;

        addCommands(
            new MoveElevatorCommand(elevator),
            new TranslationCommand(new Translation2d(0, 1), MOVE_SPEED, translational),
            new ClimbCommand(elevator)
        );

        addRequirements(translational, rotational, elevator);
    }   

}
