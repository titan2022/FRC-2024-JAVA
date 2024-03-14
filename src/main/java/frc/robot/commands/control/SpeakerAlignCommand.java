// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.utility.Localizer;
// import frc.robot.utility.Constants.Unit.*;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.RotationalDrivebase;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TranslationalDrivebase;

// import static frc.robot.utility.Constants.Unit.METERS;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /** An example command that uses an example subsystem. */
// @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
// public class SpeakerAlignCommand extends ParallelCommandGroup {
//     public SpeakerAlignCommand(RotationalDrivebase rotational, ShooterSubsystem shooter, Localizer localizer) {
//         addCommands(
//             new TeleShooterAlignCommand(shooter, localizer),
//             new TeleRotateAlignCommand(rotational, localizer)
//         );
//     }

// }
