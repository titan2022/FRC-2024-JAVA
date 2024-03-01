// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class RotateToAMPCommand extends RotationCommand implements VariantCommand {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public static final int BLUE_AMP_APRILTAG = 6;
    public static final int RED_AMP_APRILTAG = 5;
    public static final Rotation2d ROTATIONAL_SPEED = new Rotation2d(1);
    public static final Rotation2d TAG_DEADBAND = Rotation2d.fromDegrees(1);
    public int targetAprilTag;

    public RotateToAMPCommand(RotationalDrivebase driveBase, Localizer localizer, Alliance alliance) {
        super(driveBase, localizer);

        if (alliance == Alliance.Blue)
            targetAprilTag = BLUE_AMP_APRILTAG;
        else  
            targetAprilTag = RED_AMP_APRILTAG;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetAngle = localizer.getTagHeading(targetAprilTag);
        omega = new Rotation2d(Math.copySign(ROTATIONAL_SPEED.getRadians(), targetAngle.getRadians()));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(localizer.getTagHeading(targetAprilTag).getRadians()) < TAG_DEADBAND.getRadians())
            return true;
        else 
            return false;
    }


    @Override
    public void changeColorSide() {
        if (targetAprilTag == RED_AMP_APRILTAG)
            targetAprilTag = BLUE_AMP_APRILTAG;
        else if (targetAprilTag == BLUE_AMP_APRILTAG)
            targetAprilTag = RED_AMP_APRILTAG;
    }
}
