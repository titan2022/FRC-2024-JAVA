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
public class MoveToAMPCommand extends TranslationCommand implements VariantCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_AMP_APRILTAG = 6;
    public static final int RED_AMP_APRILTAG = 5;
    public static final double SPEED = 1;
    public int targetAprilTag;
    public Localizer localizer;

    public MoveToAMPCommand(TranslationalDrivebase driveBase, Localizer localizer, Alliance alliance) {
        super(driveBase);
        this.localizer = localizer;

        if (alliance == Alliance.Blue)
            targetAprilTag = BLUE_AMP_APRILTAG;
        else  
            targetAprilTag = RED_AMP_APRILTAG;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Translation2d position = localizer.getTagPosition(targetAprilTag);
        velocity = Utility.scaleVector(position, SPEED);
        time = position.getNorm() / SPEED;
        endTime = Timer.getFPGATimestamp() + time;
    }

    @Override
    public void changeColorSide() {
        if (targetAprilTag == RED_AMP_APRILTAG)
            targetAprilTag = BLUE_AMP_APRILTAG;
        else if (targetAprilTag == BLUE_AMP_APRILTAG)
            targetAprilTag = RED_AMP_APRILTAG;
    }
}
