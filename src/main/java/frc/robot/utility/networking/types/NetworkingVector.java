package frc.robot.utility.networking.types;

import edu.wpi.first.math.geometry.Translation3d;

public class NetworkingVector {
    public String objectName;
    public Translation3d vector;

    public NetworkingVector(String objectName, Translation3d vector) {
        this.objectName = objectName;
        this.vector = vector;
    }
}