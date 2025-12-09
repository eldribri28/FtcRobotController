package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.FLYWHEEL_DIAMETER_METERS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.VELOCITY_TRANSFER_EFFICIENCY;

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
     * <b>Yi</b> is Initial Height in meters,
     * <b>Yf</b> is Final Height in meters,
     * <b>targetDistance</b> is X Component of distance to target
     */
    public static LaunchResult getLaunchData(double Yi, double Yf, double targetDistance) {

        double lowestVelocity = 0;
        double bestAngle = 0;
        double bestVelocity = 0;
        double bestYVelocity = 0;
        double bestXVelocity = 0;
        double theta = 45; // Deg
        double endAngle = 72; // Deg

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
                    double Vfy = (Vi * Math.sin(theta) - g * t);
                    double Vfx = (Vi * Math.cos(theta));
                    double Vf =  Math.sqrt(Math.pow(Vfx, 2) + Math.pow(Vfy, 2));  // Landing Velocity in m/s
                    if (Vf < lowestVelocity || bestYVelocity == 0) { // If the calculated Y component of the landing velocity is less than the current best landing velocity
                        lowestVelocity = Vf;
                        bestYVelocity = Vfy;
                        bestXVelocity = Vfx;
                        bestVelocity = Vi;
                        bestAngle = theta;
                    }
                }
            }

            theta = theta + 1;

        }

        double flyWheelRpm = 0;
        if (bestAngle > 0) {
            flyWheelRpm = getFlywheelRpm(bestVelocity);
        }

        return new LaunchResult(bestAngle, bestVelocity, bestYVelocity, bestXVelocity, flyWheelRpm, lowestVelocity);

    }

    public static double getFlywheelRpm(double artifactVelocity) {
        return (artifactVelocity / ((FLYWHEEL_DIAMETER_METERS / 2) * VELOCITY_TRANSFER_EFFICIENCY) * 60) / (2 * Math.PI);
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
        private final double launchYVelocity;
        private final double launchXVelocity;
        private final double flyWheelRpm;
        private final double landingVelocity;

        public LaunchResult(double launchAngle, double launchVelocity, double launchYVelocity, double launchXVelocity, double flyWheelRpm, double landingVelocity) {
            this.launchAngle = launchAngle;
            this.launchVelocity = launchVelocity;
            this.launchYVelocity = launchYVelocity;
            this.launchXVelocity = launchXVelocity;
            this.flyWheelRpm = flyWheelRpm;
            this.landingVelocity = landingVelocity;
        }

        public double getLaunchAngle() {
            return launchAngle;
        }

        public double getLaunchVelocity() {
            return launchVelocity;
        }

        public double getLaunchYVelocity() {
            return launchYVelocity;
        }
        public double getLaunchXVelocity() {
            return launchXVelocity;
        }
        public double getFlywheelRpm() {
            return flyWheelRpm;
        }
        public double getLandingVelocity() {
            return landingVelocity;
        }

        @Override
        public String toString() {
            return "LaunchResult{" +
                    "launchAngle=" + launchAngle +
                    ", launchVelocity=" + launchVelocity +
                    ", launchYVelocity=" + launchYVelocity +
                    ", launchXVelocity=" + launchXVelocity +
                    ", flyWheelRpm=" + flyWheelRpm +
                    ", landingVelocity=" + landingVelocity +
                    '}';
        }

    }

}
