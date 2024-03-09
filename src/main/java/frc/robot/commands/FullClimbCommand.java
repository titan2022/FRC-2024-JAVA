// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TranslationalDrivebase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class FullClimbCommand extends SequentialCommandGroup{
    public static double SHOOT_SPEAKER_SPEED = 1;
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public FullClimbCommand() {

    }

}
