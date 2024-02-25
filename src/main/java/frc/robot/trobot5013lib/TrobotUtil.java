// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trobot5013lib;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class TrobotUtil {
	public static boolean withinTolerance(double value, double target, double tolerance) {
		return Math.abs(target - value) <= tolerance;
	}

	public static double modifyAxis(double value, double deadband) {

		return modifyAxis(value, 1, deadband);
	}

	public static double modifyAxis(double value, int exponent, double deadband) {
		// Deadband
		value = MathUtil.applyDeadband(value, deadband);

		value = Math.copySign(Math.pow(value, exponent), value);

		return value;
	}
	
	public static double deadband(double min, double value) {
		// sets value to 0 if below min, otherwise normalizes between min and +/- 1
		if (Math.abs(value) < min) {
			value = 0;
		} else if (value > 0) { // positive 1 is max
			value = (value - min)/(min-1);
		} else { // negative 1 is max
			value = (value - min)/(min+1);
		}

		return value;
	}
}
