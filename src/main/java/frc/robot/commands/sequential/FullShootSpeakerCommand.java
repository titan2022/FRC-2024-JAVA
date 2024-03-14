package frc.robot.commands.sequential;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.commands.align.AlignSpeakerCommand;
import frc.robot.commands.shooter.RotateShooterCommand;
import frc.robot.commands.shooter.ShooterAlignSpeakerCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.subsystems.drive.TranslationalDrivebase;

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
    public static double COMMAND_TIMER = 0;

    public static Rotation2d calculateShootAngle(Translation2d position) {
        Translation2d speakerDistace = new Translation2d(position.getNorm(), SPEAKER_HEIGHT);
        return speakerDistace.getAngle();
    }

    public static double calculateShootSpeed(Translation2d position) {
        return 0.4;
    }

    public FullShootSpeakerCommand(RotationalDrivebase rotational, ShooterSubsystem shooter, Localizer localizer) {
        addCommands(
            new AlignSpeakerCommand(rotational, localizer)
            // new ShooterAlignSpeakerCommand(shooter, localizer)
        );

        addRequirements(rotational, shooter);
    }

}
