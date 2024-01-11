package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A drivebase capable of omnidirectional translational motion.
 */
public interface TranslationalDrivebase extends Subsystem {
    /**
     * Sets the velocity in meters per second.
     * 
     * @param velocity  The desired velocity in meters per second.
     */
    public void setVelocity(Translation2d velocity);
    /**
     * Returns the current velocity in meters per second.
     * 
     * @return  The current velocity in meters per second.
     */
    public Translation2d getVelocity();
}
