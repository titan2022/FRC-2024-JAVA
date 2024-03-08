// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ShootAMPCommand extends SequentialCommandGroup{
    public static double AMP_HEIGHT = 0;
    public static double SHOOT_AMP_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShootAMPCommand(ElevatorSubsystem subsystem) {
        addCommands(
            new MoveElevatorCommand(AMP_HEIGHT, subsystem),
            new IndexOutNoteCommand(SHOOT_AMP_SPEED, subsystem)
        );

        addRequirements(subsystem);
    }

}
