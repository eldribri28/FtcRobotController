package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ACCELERATION_DUE_TO_GRAVITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.FLYWHEEL_DIAMETER_METERS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MIN_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.VELOCITY_TRANSFER_EFFICIENCY;

import java.util.ArrayList;
import java.util.List;

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

public class LaunchCalculator {

    /**
     * <b>Yi</b> is Initial Height in meters,
     * <b>Yf</b> is Final Height in meters,
     * <b>targetDistance</b> is X Component of distance to target
     */
    public static LaunchResult getLaunchData(double Yi, double Yf, double targetDistance, double currentFlyWheelRPM, double targetCloseRate) {

        double lowestVelocity = 0;
        double bestAngle = 0;
        double bestVelocity = 0;
        double bestYVelocity = 0;
        double bestXVelocity = 0;
        double timeOfFlight = 0;
        double theta = MIN_LAUNCH_ANGLE; // Deg
        double endAngle = MAX_LAUNCH_ANGLE; // Deg

        while (theta <= endAngle) {

            double denom = (2 * Math.pow(Math.cos(theta), 2) * (targetDistance * Math.tan(theta) + (Yi - Yf)));
            if (denom != 0) {
                double Vi = Math.sqrt((ACCELERATION_DUE_TO_GRAVITY * Math.pow(targetDistance, 2)) / denom);
                double subCal = Math.sqrt(Math.pow((Vi * Math.sin(theta)), 2) - 4 * (-ACCELERATION_DUE_TO_GRAVITY / 2) * (Yi - Yf));
                double subCal2 = -(Vi * Math.sin(theta));
                double t1 = (subCal2 + subCal) / (2 * (-ACCELERATION_DUE_TO_GRAVITY / 2));
                double t2 = (subCal2 - subCal) / (2 * (-ACCELERATION_DUE_TO_GRAVITY / 2));
                List<Double> times = new ArrayList<>();
                times.add(t1);
                times.add(t2);
                //double t = Math.max(Math.abs(t1), Math.abs(t2)); // Take the larger time
                for (Double t : times) {
                    if (t != 0) {
                        double Vfy = (Vi * Math.sin(theta) - ACCELERATION_DUE_TO_GRAVITY * t);
                        double Vfx = (Vi * Math.cos(theta)) - targetCloseRate;
                        double Vf = Math.sqrt(Math.pow(Vfx, 2) + Math.pow(Vfy, 2));  // Landing Velocity in m/s
                        if (Vf < lowestVelocity || bestYVelocity == 0) { // If the calculated Y component of the landing velocity is less than the current best landing velocity
                            lowestVelocity = Vf;
                            bestYVelocity = Vfy;
                            bestXVelocity = Vfx;
                            bestVelocity = Vi;
                            bestAngle = theta;
                        }
                    }
                }
            }

            theta = theta + 1;

        }

        double flyWheelRpm = 0;
        if (bestAngle > 0) {
            flyWheelRpm = getFlywheelRpm(bestVelocity);
        }
        double flyWheelAngle = getAngleForFlywheel(Yi, Yf, currentFlyWheelRPM, targetDistance);

        timeOfFlight = calculateTimeOfFlight(bestVelocity, flyWheelAngle, Yi, Yf);

        return new LaunchResult(bestAngle, bestVelocity, bestYVelocity, bestXVelocity, flyWheelRpm, lowestVelocity, flyWheelAngle, timeOfFlight);

    }

    public static double getFlywheelRpm(double artifactVelocity) {
        return (artifactVelocity / ((FLYWHEEL_DIAMETER_METERS / 2) * VELOCITY_TRANSFER_EFFICIENCY) * 60) / (2 * Math.PI);
    }

    public static double getAngleForFlywheel(double Yi, double Yf, double flyWheelRpm, double distance) {

        boolean solutionViable1 = true;
        boolean solutionViable2 = true;

        double velocity = calculateVelocity(flyWheelRpm);

        double root = Math.pow(velocity, 4) - ACCELERATION_DUE_TO_GRAVITY * (ACCELERATION_DUE_TO_GRAVITY * distance * distance + 2 * velocity * velocity * (TARGET_HEIGHT - LAUNCH_HEIGHT));
        double launchAngle1 = Math.toDegrees(Math.atan2(((velocity * velocity) - Math.sqrt(root)), (ACCELERATION_DUE_TO_GRAVITY * distance)));
        double launchAngle2 = Math.toDegrees(Math.atan2(((velocity * velocity) + Math.sqrt(root)), (ACCELERATION_DUE_TO_GRAVITY * distance)));

        // Solution is Not Viable if launch angle solution is greater than 90 degrees, less than 0 degrees, NaN, or results in a Max Height greater than 1.524m
        if (launchAngle1 < MIN_LAUNCH_ANGLE || launchAngle1 > MAX_LAUNCH_ANGLE || Double.isNaN(launchAngle1)) {
            launchAngle1 = 0;
        }
        if (launchAngle2 < MIN_LAUNCH_ANGLE || launchAngle2 > MAX_LAUNCH_ANGLE || Double.isNaN(launchAngle2)) {
            launchAngle2 = 0;
        }

        return determinePreferredLaunchResult(launchAngle1, launchAngle2);

    }
    private static double determinePreferredLaunchResult(double launchAngle1, double launchAngle2) {
        double preferredLaunchResult = 0;
        if (launchAngle1 > 0 && launchAngle2 > 0) {
            preferredLaunchResult = Math.max(launchAngle1, launchAngle2);
        } else if (launchAngle1 > 0) {
            preferredLaunchResult = launchAngle1;
        } else if (launchAngle2 > 0) {
            preferredLaunchResult = launchAngle2;
        }
        return preferredLaunchResult;
    }
    public static double calculateVelocity(double flywheelRPM) {
        return (((flywheelRPM * (2 * Math.PI)) / 60) * (FLYWHEEL_DIAMETER_METERS / 2)) * VELOCITY_TRANSFER_EFFICIENCY;
    }

    public static double calculateTimeOfFlight(double velocity, double angle, double Yi, double Yf) {
        // Convert angle to radians for Math functions
        double radians = Math.toRadians(angle);

        // Initial vertical velocity component
        double vYi = velocity * Math.sin(radians);

        // Net vertical displacement (Δy = y_final - y_initial)
        double deltaY = Yf - Yi;

        // Using Quadratic Formula: 0 = -0.5 * g * t^2 + vYi * t - deltaY
        // form: a*t^2 + b*t + c = 0
        double a = -0.5 * ACCELERATION_DUE_TO_GRAVITY;
        double b = vYi;
        double c = -deltaY;

        // Discriminant: b^2 - 4ac
        double discriminant = (b * b) - (4 * a * c);

        if (discriminant < 0) {
            // Target height is physically unreachable
            return -1.0;
        }

        // Solving for t: (-b ± sqrt(discriminant)) / 2a
        // We want the later time (when the projectile is falling)
        double t = (-b - Math.sqrt(discriminant)) / (2 * a);

        return t;
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
        private final double flyWheelAngle;
        private final double timeOfFlight;

        public LaunchResult(double launchAngle, double launchVelocity, double launchYVelocity, double launchXVelocity, double flyWheelRpm, double landingVelocity, double flyWheelAngle, double timeOfFlight) {
            this.launchAngle = launchAngle;
            this.launchVelocity = launchVelocity;
            this.launchYVelocity = launchYVelocity;
            this.launchXVelocity = launchXVelocity;
            this.flyWheelRpm = flyWheelRpm;
            this.landingVelocity = landingVelocity;
            this.flyWheelAngle = flyWheelAngle;
            this.timeOfFlight = timeOfFlight;
        }

        public double getLaunchAngle() {
            return launchAngle;
        }
        public double getLaunchVelocity() {
            return launchVelocity;
        }
        public double getLaunchYVelocity() { return launchYVelocity; }
        public double getLaunchXVelocity() {
            return launchXVelocity;
        }
        public double getFlywheelRpm() {
            return flyWheelRpm;
        }
        public double getLandingVelocity() {
            return landingVelocity;
        }
        public double getFlyWheelAngle() {
            return flyWheelAngle;
        }
        public double getTOF() {
            return timeOfFlight;
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

    public static double calculateTransitTime(double distance, double velocity, double launchAngle) {
        return (distance / (velocity * Math.cos(Math.toRadians(launchAngle)))); // Assuming no air resistance
    }
    private static double calculateMaxHeight(double velocity, double launchAngle) {
        return ((Math.pow(velocity, 2) * (Math.sin(launchAngle) * Math.sin(launchAngle))) / (2 * ACCELERATION_DUE_TO_GRAVITY)) + LAUNCH_HEIGHT;
    }


}
