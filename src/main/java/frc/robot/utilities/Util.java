package frc.robot.utilities;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double v, double maxMagnitude) {
        return inRange(v, -maxMagnitude, maxMagnitude);
    }

    /**
     * Checks if the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double v, double min, double max) {
        return v > min && v < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double normalizeAngle0To360(double currentAccumAngle) {
        // reduce the angle
        double normalizedAngle = currentAccumAngle % 360;

        // force it to be the positive remainder, so that 0 <= angle < 360
        normalizedAngle = (normalizedAngle + 360) % 360;

        return normalizedAngle;
    }

    public static double normalizeAngle180ToMinus180(double currentAccumAngle) {
        // reduce the angle
        double normalizedAngle = normalizeAngle0To360(currentAccumAngle);

        // force into the minimum absolute value residue class, so that -180 < angle <= 180
        if (normalizedAngle > 180) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }

    public static double normalizeAngle90ToMinus270(double currentAccumAngle) {
        // reduce the angle
        double normalizedAngle = normalizeAngle0To360(currentAccumAngle);

        // force into the minimum absolute value residue class, so that -270 < angle <= 90
        if (normalizedAngle > 90) {
            normalizedAngle -= 360;
        }

        return normalizedAngle;
    }

}
