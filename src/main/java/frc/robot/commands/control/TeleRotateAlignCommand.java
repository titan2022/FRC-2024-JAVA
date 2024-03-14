// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.control;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.utility.Constants;
// import frc.robot.subsystems.RotationalDrivebase;
// import frc.robot.utility.Localizer;

// /** An example command that uses an example subsystem. */
// public class TeleRotateAlignCommand extends Command {
//     @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
//     public static final int BLUE_SPEAKER_APRILTAG = 7;
//     public static final int RED_SPEAKER_APRILTAG = 4;

//     public static final Rotation2d ROTATIONAL_SPEED = new Rotation2d(0.5);

//     public RotationalDrivebase rotational;
//     public Localizer localizer;

//     public TeleRotateAlignCommand(RotationalDrivebase rotationalDrive, Localizer localizer) {
//         this.rotational = rotationalDrive;
//         this.localizer = localizer;
    
//         addRequirements(rotationalDrive);
//     }

//         @Override
//     public void initialize() {
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         Rotation2d targetAngle;
//         if (Constants.getColor() == Alliance.Blue) {
//             targetAngle = localizer.getTagHeading(BLUE_SPEAKER_APRILTAG);
//         } else {
//             targetAngle = localizer.getTagHeading(RED_SPEAKER_APRILTAG);
//         }

//         rotational.setRotationalVelocity(targetAngle);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         rotational.setRotationalVelocity(new Rotation2d(0));
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
