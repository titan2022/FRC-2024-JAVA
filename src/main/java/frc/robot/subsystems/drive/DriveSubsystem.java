package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A drivebase.
 * 
 * <p>
 * Due to how WPI handles resource locking, Translational and Rotational are
 * implemented as two sub-subsystems so that each can be controlled indidually
 */
public interface DriveSubsystem extends Subsystem{
    public TranslationalDrivebase getTranslational();

    public RotationalDrivebase getRotational();

    public void coast();

    public void brake();
}
