// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullShootSpeakerCommand extends SequentialCommandGroup {
    public static final double SHOOT_SPEAKER_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullShootSpeakerCommand(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, ElevatorSubsystem elevator) {
        addCommands(
            new RotateSpeakerCommand(Rotation2d.fromDegrees(20), shooter),
            new ShootSpeakerCommand(SHOOT_SPEAKER_SPEED, shooter, indexer, elevator)
        );

        addRequirements(shooter, elevator);
    }

}
