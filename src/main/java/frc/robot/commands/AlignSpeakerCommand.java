// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utility.Constants;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

/** An example command that uses an example subsystem. */
public class AlignSpeakerCommand extends SequentialCommandGroup implements VariantCommand{
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public Command[] commands;

    public AlignSpeakerCommand(TranslationalDrivebase translationalDrive, RotationalDrivebase rotationalDrive, Localizer localizer, Alliance alliance) {
        commands = new Command[] {
            new RotateToSpeakerCommand(rotationalDrive, localizer, alliance),
            new MoveToSpeakerCommand(translationalDrive, localizer, alliance)
        };
        
        addCommands(
            commands[0],
            commands[1]
        );

        addRequirements(translationalDrive, rotationalDrive);
    }

    @Override
    public void changeColorSide() {
        ((VariantCommand) commands[0]).changeColorSide();
        ((VariantCommand) commands[1]).changeColorSide();
    }

}
