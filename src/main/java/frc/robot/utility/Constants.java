// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Unit conversion constants to standard units.
   * 
   * <p>To convert into standard units, multiply by the appropriate constant.
   * To convert from standard units, divide by the appropriate constant.
   * 
   * <p>Examples:
   * <p>Save 26 inches as meters: {@code double length = 26 * IN;}
   * <p>Convert 1.5 radians to Falcon counts: {@code 1.5 / FALCON_TICKS}
   */
  public static final class Unit {
    public static final double METERS = 1.0, M = 1.0;
    public static final double CM = 0.01, MM = 0.001;
    public static final double IN = Units.inchesToMeters(1);
    public static final double FT = Units.feetToMeters(1);
    public static final double RAD = 1.0;
    public static final double DEG = Units.degreesToRadians(1);
    public static final double ROT = Math.PI * 2;
    public static final double FALCON_CPR = 2048;
    public static final double CANCODER_CPR = 4096;
    public static final double FALCON_TICKS = ROT / FALCON_CPR;
    public static final double CANCODER_TICKS = ROT / CANCODER_CPR;
    public static final double SECONDS = 1.0, S = 1.0;
    public static final double MIN = 1.0/60;
    public static final double MS = 0.001;
  }
}
