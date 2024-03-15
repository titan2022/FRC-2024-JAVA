package frc.robot.commands.auto;

import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.RotationCommand;
import frc.robot.commands.drive.TranslationCommand;
import frc.robot.commands.shooter.RotateShooterCommand;
import frc.robot.commands.shooter.SimpleShootCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class SimpleAutoPlanRight extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static double SHOOT_SPEAKER_SPEED = 0.5;
    public static final double SPEAKER_HEIGHT = 2 * METERS;
    public static Rotation2d SHOOT_ANGLE = Rotation2d.fromDegrees(60);

    public SimpleAutoPlanRight(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, ElevatorSubsystem elevator, Localizer localizer) {
        double sign = 1;
        // if (Constants.getColor() == Alliance.Blue) {    
        //     sign = 1;
        // } else 
        //     sign = -1;
        
        addCommands(
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(45).times(sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 2), 1, translational)
        );

        // addRequirements(rotational, translational, shooter, indexer, elevator, intake);
    }

}
