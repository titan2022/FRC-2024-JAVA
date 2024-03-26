// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import frc.robot.utility.Constants.Unit;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Constants;
import frc.robot.utility.Localizer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ShooterSpeakerAlgCommand extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final double GRAVITY_STRENGTH = 9.8 * Unit.METERS;
    public static final double SPEAKER_HEIGHT = 2.0 * Unit.METERS;
    public static final Translation2d SHOOTER_PIVOT_OFFSET = new Translation2d(0.95 * Unit.IN, 7.89 * Unit.IN); 
    public static final double SHOOTER_ARM_LENGTH = 8.387 * Unit.IN;
    public static final Translation2d TARGET_OFFSET = new Translation2d();
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;

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

        Translation2d robotDistanceToAprilTag = new Translation2d(horizontalDistance, SPEAKER_HEIGHT);
        Translation2d shootVector = robotDistanceToAprilTag.minus(SHOOTER_PIVOT_OFFSET);
        Rotation2d dynamicStartAngle = Rotation2d.fromDegrees(55);
        // Translation2d dynamicShooterStartPoint = new Translation2d(dynamicStartAngle.getCos() * SHOOTER_ARM_LENGTH, dynamicStartAngle.getSin() * SHOOTER_ARM_LENGTH);
        Translation2d dynamicShooterStartPoint = new Translation2d(0, 0);

        Translation2d trueShootVector = shootVector.minus(dynamicShooterStartPoint);
        return trueShootVector.plus(TARGET_OFFSET);
    }

    public ShooterSpeakerAlgCommand(double speed, ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer) {
        Translation2d shootTarget = getShootVector(localizer);
        SmartDashboard.putNumber("Shoot X", shootTarget.getX());
        SmartDashboard.putNumber("Shoot Y", shootTarget.getY());

        Rotation2d setAngle;
        try {
            setAngle = calculateAngle(speed, shootTarget);
        } catch (Exception e) {
            setAngle = Rotation2d.fromDegrees(65);
        }        

        SmartDashboard.putNumber("Shoot Angle", setAngle.getDegrees());

        addCommands(
            new RotateShooterCommand(setAngle, shooter),
            new RevShooterCommand(speed, shooter),
            new FireShooterCommand(indexer, shooter)
        );

    }
}

