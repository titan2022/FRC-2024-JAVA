package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;

public class Utility {
    public static Translation2d scaleVector(Translation2d vec, double magnitude) {
        return unitVector(vec).times(magnitude);
    }
    
    public static Translation2d unitVector(Translation2d vec) {
        return vec.div(vec.getNorm());
    }
}
