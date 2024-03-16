package frc.robot.commands.auto;

import static frc.robot.utility.Constants.Unit.METERS;
import static frc.robot.utility.Constants.Unit.SECONDS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.control.NoteIntakeCommand;
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
import frc.robot.utility.Constants;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class SimpleAutoPlanEthan extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double SHOOT_SPEAKER_SPEED = 0.5;
    public static final double SPEAKER_HEIGHT = 2 * METERS;
    public static final Rotation2d SHOOT_ANGLE = Rotation2d.fromDegrees(64.9);
    public static final double WAIT_LENGTH = 1 * SECONDS;

    public SimpleAutoPlanEthan(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, ElevatorSubsystem elevator, Localizer localizer) {
        // double sign = 1;
        double sign;
        if (Constants.getColor() == Alliance.Blue) {    
            sign = 1;
        } else 
            sign = -1;

        SmartDashboard.putBoolean("1", true);
        
        addCommands(
            //// Shoot the preloaded note ////
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new WaitCommand(WAIT_LENGTH),
            //// Eat 1st note ////
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            // Turn 30 degrees clockwise
            new RotationCommand(Rotation2d.fromDegrees(30).times(sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 2), 1, translational),
            new NoteIntakeCommand(indexer, intake, shooter),
            new TranslationCommand(new Translation2d(sign * 0, -2), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(30).times(-sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            //// Shoot 1st note ////
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new WaitCommand(WAIT_LENGTH),
            //// Eat 2nd note ////
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(45).times(sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 2), 1, translational),
            new NoteIntakeCommand(indexer, intake, shooter),
            new TranslationCommand(new Translation2d(sign * 0, -2), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(45).times(-sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            //// Shoot 2nd note ////
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new WaitCommand(WAIT_LENGTH),
            //// Eat 3rd note ////
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(45).times(sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 1.8), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(30).times(sign), rotational, localizer),
            new NoteIntakeCommand(indexer, intake, shooter),
            new TranslationCommand(new Translation2d(sign * 0, -1.8), 1, translational),
            new RotationCommand(Rotation2d.fromDegrees(45).times(-sign), rotational, localizer),
            new RotationCommand(Rotation2d.fromDegrees(30).times(-sign), rotational, localizer),
            new TranslationCommand(new Translation2d(sign * 0, 0.5), 1, translational),
            //// Shoot 3rd note ////
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new WaitCommand(WAIT_LENGTH)
        );


    }

}
