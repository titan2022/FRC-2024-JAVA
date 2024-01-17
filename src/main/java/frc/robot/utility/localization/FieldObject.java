package frc.robot.utility.localization;

import edu.wpi.first.math.geometry.Translation2d;

public interface FieldObject {
    public FieldObjectType getType();
    public String getName();
    public void setName(String name);
    public Translation2d getPosition();
    public void setPosition(Translation2d position);
}