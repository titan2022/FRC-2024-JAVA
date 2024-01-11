package frc.robot.utility;

public interface FieldObject {
    public FieldObjectType getType();

    public void setName(String name);

    public Vector2D getPosition();

    public void setPosition(Vector2D position);
}