package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A drivebase capable of rotation.
 */
public interface RotationalDrivebase extends Subsystem {
    /**
     * Sets the rotational velocity in radians per second.
     * 
     * @param omega  The desired rotational velocity in radians per second.
     */
    public void setRotation(double omega);
    /**
     * Returns the current rotational velocity in radians per second.
     * 
     * @return  The current rotational velocity in radians per second.
     */
    public double getRate();
}
