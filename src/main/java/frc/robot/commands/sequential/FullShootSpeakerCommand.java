package frc.robot.commands.sequential;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

import frc.robot.utility.Localizer;

// package frc.robot.commands;

import frc.robot.utility.Constants.Unit.*;
import frc.robot.commands.align.AlignSpeakerCommand;
import frc.robot.commands.shooter.FireShooterCommand;
import frc.robot.commands.shooter.RevShooterCommand;
import frc.robot.commands.shooter.ShooterSpeakerAlignCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.TranslationalDrivebase;

import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullShootSpeakerCommand extends SequentialCommandGroup {
    public static final double SHOOT_SPEAKER_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullShootSpeakerCommand(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer) {
        
        addCommands(
            new ParallelCommandGroup(
                new ShooterSpeakerAlignCommand(SHOOT_SPEAKER_SPEED, shooter, localizer),
                new AlignSpeakerCommand(rotational, localizer),
                new RevShooterCommand(SHOOT_SPEAKER_SPEED, shooter)
            ),
            new FireShooterCommand(indexer, shooter)
        );

        addRequirements(translational);
    }

}
