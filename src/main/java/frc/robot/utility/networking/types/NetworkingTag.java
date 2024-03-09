package frc.robot.utility.networking.types;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class NetworkingTag {
    public String objectName;
    public Translation3d position;
    public Rotation3d rotation;
    public int id;

    public NetworkingTag(String objectName, Translation3d position, Rotation3d rotation, int id) {
        this.objectName = objectName;
        this.position = position;
        this.rotation = rotation;
        this.id = id;
    }

    public String toString() {
        return id + ": " + position.toString() + " " + rotation.toString();
    }
}