// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;
import frc.robot.utility.Utility;

/** An example command that uses an example subsystem. */
public class MoveToSpeakerCommand extends TranslationCommand implements VariantCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_SPEAKER_APRILTAG = 7;
    public static final int RED_SPEAKER_APRILTAG = 4;
    public static final double SPEED = 1;
    public static final double SUBWOOFER_LENGTH = 0.91;

    public int targetAprilTag;
    public Localizer localizer;

    public MoveToSpeakerCommand(TranslationalDrivebase driveBase, Localizer localizer, Alliance alliance) {
        super(driveBase);
        this.localizer = localizer;

        if (alliance == Alliance.Blue)
            targetAprilTag = BLUE_SPEAKER_APRILTAG;
        else  
            targetAprilTag = RED_SPEAKER_APRILTAG;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Translation2d position = localizer.getTagPosition(targetAprilTag);
        position.minus(new Translation2d(0, SUBWOOFER_LENGTH));
        velocity = Utility.scaleVector(position, SPEED);
        time = position.getNorm() / SPEED;
        endTime = Timer.getFPGATimestamp() + time;
    }

    @Override
    public void changeColorSide() {
        if (targetAprilTag == RED_SPEAKER_APRILTAG)
            targetAprilTag = BLUE_SPEAKER_APRILTAG;
        else if (targetAprilTag == BLUE_SPEAKER_APRILTAG)
            targetAprilTag = RED_SPEAKER_APRILTAG;
    }
}
