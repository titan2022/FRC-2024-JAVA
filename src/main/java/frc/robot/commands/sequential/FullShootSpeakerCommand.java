// package frc.robot.commands.sequential;
// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

<<<<<<< HEAD
// import frc.robot.utility.Localizer;

// // package frc.robot.commands;

// import frc.robot.utility.Constants.Unit.*;
// import frc.robot.commands.shooter.FireShooterCommand;
// import frc.robot.commands.shooter.RevShooterCommand;
// import frc.robot.commands.shooter.ShooterAlignSpeakerCommand;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.subsystems.drive.RotationalDrivebase;
// import frc.robot.subsystems.ShooterSubsystem;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.units.Unit;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /** An example command that uses an example subsystem. */
// public class FullShootSpeakerCommand extends SequentialCommandGroup {
//     public static final double SHOOT_SPEAKER_SPEED = 8;
//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     public FullShootSpeakerCommand(RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer) {
//         addCommands(
//             new ShooterAlignSpeakerCommand(19.5, shooter, localizer),
//             new RevShooterCommand(SHOOT_SPEAKER_SPEED, shooter),
//             new FireShooterCommand(indexer, shooter)
//         );
//     }
// }
=======
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.FireShooterCommand;
import frc.robot.commands.shooter.RevShooterCommand;
import frc.robot.commands.shooter.ShooterAlignSpeakerCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class FullShootSpeakerCommand extends SequentialCommandGroup {
    public static final double SHOOT_SPEAKER_SPEED = 8;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullShootSpeakerCommand(RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer, LEDSubsystem led) {
        addCommands(
            new ShooterAlignSpeakerCommand(19.5, shooter, localizer),
            new RevShooterCommand(SHOOT_SPEAKER_SPEED, shooter, led),
            new FireShooterCommand(indexer, shooter, led)
        );
    }
}
>>>>>>> a7e1b18f53f824f514faeac5d21bf30b7500cac7
