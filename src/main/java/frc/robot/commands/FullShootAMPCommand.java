// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullShootAMPCommand extends SequentialCommandGroup {
    public static final double SHOOT_AMP_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullShootAMPCommand(ElevatorSubsystem elevator, IndexerSubsystem indexer) {
        addCommands(
            new MoveElevatorCommand(true, elevator),
            new ShootAMPCommand(indexer),
            new MoveElevatorCommand(false, elevator)
        );

        addRequirements(elevator);
    }

}
