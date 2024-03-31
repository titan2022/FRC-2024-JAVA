// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static frc.robot.utility.Constants.Unit.IN;
import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class ShooterSpeakerAlgCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double GRAVITY_STRENGTH = 9.8 * METERS;
    public static final double SPEAKER_HEIGHT = 2.0 * METERS;
    public static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.95 * IN, 7.89 * IN); 
    public static final double SHOOTER_ARM_LENGTH = 8.387 * IN;
    public static final Translation2d TARGET_OFFSET = new Translation2d(0.92, 0.42);

    //Assume vector from note in shooter
    public static Rotation2d calculateAngle(double speed, Translation2d shootTarget) {
        double x = shootTarget.getX();
        double y = shootTarget.getY();
        double gx2OverV = GRAVITY_STRENGTH * x * x / (speed * speed);
        //Quadratic formula 
        double middleTerm = (GRAVITY_STRENGTH * GRAVITY_STRENGTH * Math.pow(x, 4) / (Math.pow(speed, 4)));
        double lastTerm = 2 * GRAVITY_STRENGTH * x * x * y / (speed * speed);
        double sqrt = Math.sqrt(x * x - middleTerm - lastTerm);
        double numerator = x - sqrt;

        return new Rotation2d(Math.atan2(numerator, gx2OverV));
    }

    public static Rotation2d linearShooter(double speed, Translation2d shootTarget) {
        return new Rotation2d(Math.atan2(shootTarget.getX(), shootTarget.getY()));
    }


    public static Translation2d getShootVector(Localizer localizer) {
        double horizontalDistance = localizer.getSpeakerPosition().getNorm();
        // double horizontalDistance = SmartDashboard.getNumber("Speaker Distance", 0) * IN;

        Translation2d robotDistanceToAprilTag = new Translation2d(horizontalDistance, SPEAKER_HEIGHT);
        Translation2d shootVector = robotDistanceToAprilTag.minus(SHOOTER_PIVOT_OFFSET);
        // Rotation2d dynamicStartAngle = Rotation2d.fromDegrees(45);
        // Translation2d dynamicShooterStartPoint = new Translation2d(dynamicStartAngle.getCos() * SHOOTER_ARM_LENGTH, dynamicStartAngle.getSin() * SHOOTER_ARM_LENGTH);
        Translation2d dynamicShooterStartPoint = new Translation2d(0, 0);

        Translation2d trueShootVector = shootVector.minus(dynamicShooterStartPoint);
        return trueShootVector.plus(TARGET_OFFSET);
    }

    public ShooterSpeakerAlgCommand(double speed, RotationalDrivebase rotational, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer, LEDSubsystem led) {
        // Translation2d shootTarget = getShootVector(localizer);
        // Translation2d shootTarget = new Translation2d(80 * IN, SPEAKER_HEIGHT);
        // SmartDashboard.putNumber("Shoot X", shootTarget.getX());
        // SmartDashboard.putNumber("Shoot Y", shootTarget.getY());

        // Rotation2d setAngle;
        // try {
        //     setAngle = calculateAngle(speed * 1.5, shootTarget);
        // } catch (Exception e) {
        //     setAngle = Rotation2d.fromDegrees(30);
        // }        

        // SmartDashboard.putNumber("Shoot Angle", setAngle.getDegrees());
        addCommands(
            // new RevShooterCommand(speed, shooter, led),
            // new FireShooterCommand(indexer, shooter, led)
            // new AlignSpeakerCommand(rotational, localizer)
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    new RevShooterCommand(speed, shooter, led),
                    new FireShooterCommand(indexer, shooter, led)
                ), 
                new ShooterAlignSpeakerCommand(speed * 1.15, shooter, localizer)
            )
        );

        addRequirements(shooter, indexer);
    }
}

    