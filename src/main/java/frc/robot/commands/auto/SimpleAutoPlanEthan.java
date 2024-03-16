package frc.robot.commands.auto;

import static frc.robot.utility.Constants.Unit.DEG;
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

/** Starting from the subwoofer on the source side, shoot two notes. */
public class SimpleAutoPlanEthan extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double SHOOT_SPEAKER_SPEED = 0.5;
    public static final double SPEAKER_HEIGHT = 2 * METERS;
    public static final Rotation2d SHOOT_ANGLE = Rotation2d.fromDegrees(64.9);
    public static final double WAIT_LENGTH = 1 * SECONDS;

    TranslationalDrivebase translational;
    RotationalDrivebase rotational; 
    ShooterSubsystem shooter; 
    IndexerSubsystem indexer; 
    IntakeSubsystem intake; 
    ElevatorSubsystem elevator; 
    Localizer localizer;

    double sign;

    public SimpleAutoPlanEthan(TranslationalDrivebase translational, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, ElevatorSubsystem elevator, Localizer localizer) {
        this.translational = translational;
        this.rotational = rotational;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.elevator = elevator;
        this.localizer = localizer;

        if (Constants.getColor() == Alliance.Blue) {    
            sign = 1;
        } else 
            sign = -1;

        SmartDashboard.putBoolean("1", true);
        
        shootNote();
        translateRotateTranslateInOut(3*METERS, 120*DEG, 3.5*METERS, this::intakeNote);
        shootNote();
        translateRotateTranslateInOut(3*METERS, 95*DEG, 3.5*METERS, this::intakeNote);
    }

    public interface InternalCommand {
        void add();
    }

    public void shootNote() {
        addCommands(
            new RotateShooterCommand(SHOOT_ANGLE, shooter),
            new SimpleShootCommand(SHOOT_SPEAKER_SPEED, shooter, indexer),
            new WaitCommand(WAIT_LENGTH)
        );
    }

    public void rotateIn(double degrees) {
        // Since we start from the source side, inwards is counterclockwise on the blue side and clockwise on the red side.
        addCommands(
            new RotationCommand(Rotation2d.fromDegrees(degrees).times(sign), rotational, localizer)
        );
    }

    public void rotateOut(double degrees) {
        // Since we start from the source side, outwards is clockwise on the blue side and countercockwise on the red side.
        addCommands(
            new RotationCommand(Rotation2d.fromDegrees(degrees).times(-sign), rotational, localizer)
        );
    }

    public void moveForward(double translation) {
        addCommands(
            new TranslationCommand(new Translation2d(0, translation), 1, translational)
        );
    }

    public void intakeNote() {
        addCommands(
            new NoteIntakeCommand(indexer, intake, shooter)
        );
    }

    public void translateRotateTranslateInOut(double translation1, double rotation, double translation2, InternalCommand command) {
        moveForward(translation1);
        rotateIn(rotation);
        moveForward(translation2);
        command.add();
        moveForward(-translation2);
        rotateOut(rotation);
        moveForward(-translation1);
    }

}
