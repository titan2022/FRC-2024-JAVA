// package frc.robot.commands.sequential;

// import frc.robot.commands.control.MoveElevatorCommand;
// import frc.robot.commands.shooter.ShootAMPCommand;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.IndexerSubsystem;
// import frc.robot.utility.Localizer;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// /** An example command that uses an example subsystem. */
// public class FullShootAMPCommand extends SequentialCommandGroup {
//     public static final double SHOOT_AMP_SPEED = 1;
//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     public FullShootAMPCommand(ElevatorSubsystem elevator, IndexerSubsystem indexer, Localizer localizer) {
//         addCommands(
//             new MoveElevatorCommand(true, elevator),
//             new ShootAMPCommand(indexer),
//             new MoveElevatorCommand(false, elevator)
//         );

//         addRequirements(elevator, indexer);
//     }

// }
