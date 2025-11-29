package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;
/**
 * 2025-26 FTC Season DECODE
 * <br/>
 * Class for calculating launch velocity and angle required to produce the
 * lowest artifact landing velocity. This should help limit artifact bounce
 * out.
 *
 * @author Team 14739 Metal Benders
 * @version 0.1
 * @since 2025-11-29
 */

public class LaunchCalculator2 {

    /**
     * <b>Yi</b> is Inital Height in meters,
     * <b>Yf</b> is Final Height in meters,
     * <b>targetDistance</b> is X Component of distance to target
     */
    public static LaunchResult getLaunchData(double Yi, double Yf, double targetDistance) {

        double bestAngle = 0;
        double bestVelocity = 0;
        double theta = 45; // Deg
        double endAngle = 65; // Deg

        while (theta < endAngle) {

            double g = 9.8; // gravity 9.8 m/s^2
            double denom = (2 * Math.pow(Math.cos(theta), 2) * (targetDistance * Math.tan(theta) + (Yi - Yf)));
            if (denom != 0) {
                double Vi = Math.sqrt((g * Math.pow(targetDistance, 2)) / denom);
                double subCal = Math.sqrt(Math.pow((Vi * Math.sin(theta)), 2) - 4 * (-g / 2) * (Yi - Yf));
                double subCal2 = -(Vi * Math.sin(theta));
                double t1 = (subCal2 + subCal) / (2 * (-g / 2));
                double t2 = (subCal2 - subCal) / (2 * (-g / 2));
                double t = Math.max(Math.abs(t1), Math.abs(t2)); // Take the larger time
                if (t != 0) {
                    double Vf = Math.sqrt((Vi * Math.cos(theta)) / (Vi * Math.sin(theta) - g * t));  // Landing Velocity in m/s
                    if (Vf < bestVelocity || bestVelocity == 0) { // If the calculated landing velocity is less than the current best landing velocity
                        bestVelocity = Vf;
                        bestAngle = theta;
                    }
                }
            }

            theta = theta + 0.1;

        }

        return new LaunchResult(bestAngle, bestVelocity);

    }

    /**
     * LaunchResult Class returns computed Launch Angle
     * and Launch Velocity from LaunchCalculator2.
     * <br/>
     * Provides <b>getLaunchAngle()</b> and <b>getLaunchVelocity()</b> methods.
     */
    public static class LaunchResult {

        private final double launchAngle;
        private final double launchVelocity;

        public LaunchResult(double launchAngle, double launchVelocity) {
            this.launchAngle = launchAngle;
            this.launchVelocity = launchVelocity;
        }

        public double getLaunchAngle() {
            return launchAngle;
        }

        public double getLaunchVelocity() {
            return launchVelocity;
        }

        @Override
        public String toString() {
            return "LaunchResult{" +
                    "launchAngle=" + launchAngle +
                    ", launchVelocity=" + launchVelocity +
                    '}';
        }

    }

}
