package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class Utility {
    public static Translation2d normalizeVector(Translation2d vec) {
        return vec.div(vec.getNorm());
    }

    public static Translation2d scaleMagnitude(Translation2d vec, double magnitude) {
        return normalizeVector(vec).times(magnitude);
    }
}
