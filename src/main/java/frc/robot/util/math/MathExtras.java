package frc.robot.util.math;

public class MathExtras {

    public static double map(
            double value,
            double inMin, double inMax,
            double outMin, double outMax) {

        return (value - inMin) * (outMax - outMin)
                / (inMax - inMin) + outMin;
    }
}
