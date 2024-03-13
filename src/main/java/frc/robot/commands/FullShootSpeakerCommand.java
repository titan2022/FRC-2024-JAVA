// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;

import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullShootSpeakerCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double SHOOT_SPEAKER_SPEED = 0.5;
    public static final double SPEAKER_HEIGHT = 2 * METERS;

    public static Rotation2d calculateShootAngle(Translation2d position) {
        Translation2d speakerDistace = new Translation2d(position.getNorm(), SPEAKER_HEIGHT);
        return speakerDistace.getAngle();
    }

    public static double calculateShootSpeed(Translation2d position) {
        return 0.4;
    }

    public FullShootSpeakerCommand(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer) {
        addCommands(
            new RotateShooterCommand(calculateShootAngle(new Translation2d(1,1)), shooter),
            new ShootSpeakerCommand(SHOOT_SPEAKER_SPEED, shooter, indexer)
        );

        addRequirements(shooter, indexer);
    }

}
