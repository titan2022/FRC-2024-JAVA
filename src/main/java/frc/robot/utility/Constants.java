// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**
	 * Unit conversion constants to standard units.
	 * 
	 * <p>
	 * To convert into standard units, multiply by the appropriate constant.
	 * To convert from standard units, divide by the appropriate constant.
	 * 
	 * <p>
	 * Examples:
	 * <p>
	 * Save 26 inches as meters: {@code double length = 26 * IN;}
	 * <p>
	 * Convert 1.5 radians to Falcon counts: {@code 1.5 / FALCON_TICKS}
	 */
	public static final class Unit {
		public static final double METERS = 1.0, M = METERS;
		public static final double CM = 0.01 * M, MM = 0.001 * M;
		public static final double IN = 0.0254 * M;
		public static final double FT = 12 * IN;
		public static final double RADIANS = 1.0, RAD = RADIANS;
		public static final double DEG = Math.PI / 180.0 * RAD; // Degrees to radians
		public static final double ROT = Math.PI * 2 * RAD; // Rotations to radians
		public static final double FALCON_CPR = 2048;
		public static final double CANCODER_CPR = 4096;
		public static final double QUAD_ENCODER_CPR = 1024;
		public static final double FALCON_TICKS = ROT / FALCON_CPR; // Ticks to radians
		public static final double CANCODER_TICKS = ROT / CANCODER_CPR; // Ticks to radians
		public static final double QUAD_ENCODER_TICKS = ROT / QUAD_ENCODER_CPR; // Ticks to radians
		public static final double SECONDS = 1.0, S = SECONDS;
		public static final double MIN = 60 * S;
		public static final double MS = 0.001 * S;
		public static final double PCT_OUT = 1.0;
		public static final double PID_OUT = 1.0 / 1024 * PCT_OUT;
	}

	public enum LimelightPipeline {
		TAPE(0), APRILTAGS(1),
		// RED_GRID_APRILTAGS(2), BLUE_GRID_APRILTAGS(3),
		// RED_SUBSTATION_APRILTAG(4), BLUE_SUBSTATION_APRILTAG(5),
		CONES(4), CUBES(5);

		public int index;

		private LimelightPipeline(int index) {
			this.index = index;
		}
	}

	public static final double LIMELIGHT_HEIGHT = 0;
	public static final double LIMELIGHT_OFFSET = 0;
	public static final double LIMELIGHT_PITCH = 0;

	public static double stdToPrec(double std) {
		return 1 / (std * std);
	}

	public static double varToPrec(double var) {
		return 1 / var;
	}

	public static double precToStd(double prec) {
		return 1 / Math.sqrt(prec);
	}

	public static double precToVar(double prec) {
		return 1 / prec;
	}

	public static double stdToVar(double std) {
		return std * std;
	}

	public static double varToStd(double var) {
		return Math.sqrt(var);
	}

	// TODO: Measure AprilTag precisions
	public static final double APRIL_POSITION_PREC = stdToPrec(1 * Unit.CM);
	public static final double APRIL_ORIENTATION_PREC = stdToPrec(4 * Unit.DEG);

	// TODO: Measure odometry precisions
	public static final double ODOMETRY_VELOCITY_PREC = stdToPrec(2 * (Unit.CM / Unit.S));
	public static final double ODOMETRY_RATE_PREC = stdToPrec(4 * (Unit.DEG / Unit.S));

	// TODO: Measure pigeon precision
	public static final double PIGEON_ORIENTATION_PREC = stdToPrec(0.25 * Unit.DEG);
	public static final double PIGEON_RATE_PREC = stdToPrec(0.25 * Unit.DEG / Unit.S);

	// TODO: Measure max acceleration at full weight
	public static final double MAX_ACCELERATION = 5 * (Unit.M / Unit.S / Unit.S);
	public static final double MAX_ANGULAR_ACCELERATION = 1 * (Unit.ROT / Unit.S / Unit.S);

	public static final class RobotSize {
		//Units in meters
		public static final double WIDTH = 1;
		public static final double LENGTH = 1;
	}
	// public static class FeedForward 
	// {
	// 	public static final double kS = 0.015;
	// 	public static final double kV = 0.18;
	// 	public static final double kA = 0;
	// }
	/**
	 * Contains a velocity based PID configuration.
	 * 
	 * @return TalonFX Configuration Object
	 */
	public static TalonFXConfiguration getSwerveDriveTalonDriveConfig() {
		TalonFXConfiguration talon = new TalonFXConfiguration();
		// Add configs here:
		// talon.slot0.kP = 0.12;
		// talon.slot0.kI = 0;
		// talon.slot0.kD = 2.5;
		// talon.slot0.kF = 0;
		talon.slot0.kP = 0.1;
		talon.slot0.kI = 0;
		talon.slot0.kD = 0;
		talon.slot0.kF = 0;
		talon.slot0.integralZone = 900;
		talon.slot0.allowableClosedloopError = 20;
		talon.slot0.maxIntegralAccumulator = 254.000000;
		// talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the
		// PID controller
		// talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the
		// PID controller

		return talon;
	}

	/**
	 * Contains a position based PID configuration
	 * 
	 * @return TalonFX Configuration Object
	 */
	public static TalonFXConfiguration getSwerveDriveTalonRotaryConfig() {
		TalonFXConfiguration talon = new TalonFXConfiguration();
		// Add configs here:
		talon.slot0.kP = 0.38;
		talon.slot0.kI = 0;
		talon.slot0.kD = 2.0;
		talon.slot0.kF = 0;
		talon.slot0.integralZone = 75;
		talon.slot0.allowableClosedloopError = 5;// 217;
		talon.slot0.maxIntegralAccumulator = 5120;
		// talon.slot0.closedLoopPeakOutput = 0.869990; // Sets maximum output of the
		// PID controller
		// talon.slot0.closedLoopPeriod = 33; // Sets the hardware update rate of the
		// PID controller

		return talon;
	}
}