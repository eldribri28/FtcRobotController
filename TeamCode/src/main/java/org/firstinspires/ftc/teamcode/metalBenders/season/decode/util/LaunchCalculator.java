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

        double lowestLaunchVelocity = 0;
        double bestAngle = 0;
        double bestLandingVelocity = 0;
        double bestYVelocity = 0;
        double bestXVelocity = 0;
        double timeOfFlight = 0;
        double theta = MIN_LAUNCH_ANGLE; // Deg
        double endAngle = MAX_LAUNCH_ANGLE; // Deg

        //if (targetDistance >= 2.8) { targetDistance += 0.100; }

        while (theta <= endAngle) {

            double thetaRads = Math.toRadians(theta);

            double Vfy = 0;
            double Vfx = 0;
            double Vf = 0;
            double maxHeight = 0;
            double tof = 0;
            double Vi = 0;
            double denom = 0;

            denom = (2 * Math.pow(Math.cos(thetaRads), 2) * (targetDistance * Math.tan(thetaRads) - (Yf - Yi)));
            if (denom != 0) {
                Vi = Math.sqrt((ACCELERATION_DUE_TO_GRAVITY * Math.pow(targetDistance, 2)) / denom); // need to adjust for close rate only being x component
                tof = calculateTimeOfFlight(Vi, theta, Yi, Yf);
                if (tof > 0) {
                    Vfy = (Vi * Math.sin(thetaRads) - ACCELERATION_DUE_TO_GRAVITY * tof);
                    Vfx = (Vi * Math.cos(thetaRads));
                    Vf = Math.sqrt(Math.pow(Vfx, 2) + Math.pow(Vfy, 2));  // Landing Velocity in m/s
                    maxHeight = calculateMaxHeight(Vf, thetaRads);
                    if ((Vf < bestLandingVelocity || lowestLaunchVelocity == 0) && maxHeight > Yf + 0.200 && targetDistance >= 6) { // If the calculated Y component of the landing velocity is less than the current best landing velocity
                        timeOfFlight = tof;
                        lowestLaunchVelocity = Vi;
                        bestYVelocity = Vfy;
                        bestXVelocity = Vfx;
                        bestLandingVelocity = Vf;
                        bestAngle = theta;
                    } else if ((tof < timeOfFlight || lowestLaunchVelocity == 0) && maxHeight > Yf + 0.200 && targetDistance < 6) {
                        timeOfFlight = tof;
                        lowestLaunchVelocity = Vi;
                        bestYVelocity = Vfy;
                        bestXVelocity = Vfx;
                        bestLandingVelocity = Vf;
                        bestAngle = theta;
                    }
                }

            }

            theta = theta + 0.2;

        }

        double flyWheelRpm = 0;
        if (bestAngle > 0) {
            flyWheelRpm = getFlywheelRpm(lowestLaunchVelocity);
        }
        double currentFlyWheelAngle = getAngleForFlywheel(Yi, Yf, currentFlyWheelRPM, targetDistance);

        return new LaunchResult(bestAngle, lowestLaunchVelocity, bestYVelocity, bestXVelocity, flyWheelRpm, bestLandingVelocity, currentFlyWheelAngle, timeOfFlight);

    }

    public static double getFlywheelRpm(double artifactVelocity) {
        return (((artifactVelocity * 60) / VELOCITY_TRANSFER_EFFICIENCY) / (Math.PI * FLYWHEEL_DIAMETER_METERS));
    }

    public static double getAngleForFlywheel(double Yi, double Yf, double flyWheelRpm, double distance) {

        double velocity = calculateVelocity(flyWheelRpm);

        double velocity2 = Math.pow(velocity, 2);
        double velocity4 = Math.pow(velocity, 4);
        double distance2 = Math.pow(distance, 2);
        double deltaY = Yf - Yi; // Vertical displacement

        // The discriminant from the quadratic formula
        double discriminant = velocity4 - ACCELERATION_DUE_TO_GRAVITY * (ACCELERATION_DUE_TO_GRAVITY * distance2 + 2 * deltaY * velocity2);

        //  If the discriminant is negative, the target is out of reach at this velocity
        if (discriminant < 0) {
            return 0;
        }

        double sqrtDiscriminant = Math.sqrt(discriminant);

        // Calculate both possible values for tan(theta)
        double launchAngleLow = (velocity2 - sqrtDiscriminant) / (ACCELERATION_DUE_TO_GRAVITY * distance);
        double launchAngleHigh = (velocity2 + sqrtDiscriminant) / (ACCELERATION_DUE_TO_GRAVITY * distance);

        // Solution is Not Viable if launch angle solution is greater than 90 degrees, less than 0 degrees, NaN, or results in a Max Height greater than 1.524m
        if (launchAngleLow < MIN_LAUNCH_ANGLE || launchAngleLow > MAX_LAUNCH_ANGLE || Double.isNaN(launchAngleLow)) {
            launchAngleLow = 0;
        }
        if (launchAngleHigh < MIN_LAUNCH_ANGLE || launchAngleHigh > MAX_LAUNCH_ANGLE || Double.isNaN(launchAngleHigh)) {
            launchAngleHigh = 0;
        }

        return  determinePreferredLaunchResult(launchAngleLow, launchAngleHigh, distance);

    }
    private static double determinePreferredLaunchResult(double launchAngleLow, double launchAngleHigh, double distance) {
        double preferredLaunchResult = 0;

        if (distance > 2) {
            if (launchAngleLow > MIN_LAUNCH_ANGLE && launchAngleLow < MAX_LAUNCH_ANGLE && launchAngleHigh > MIN_LAUNCH_ANGLE && launchAngleHigh < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = Math.max(launchAngleLow, launchAngleHigh);
            } else if (launchAngleLow > MIN_LAUNCH_ANGLE && launchAngleLow < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = launchAngleLow;
            } else if (launchAngleHigh > MIN_LAUNCH_ANGLE && launchAngleHigh < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = launchAngleHigh;
            } else {
                preferredLaunchResult = launchAngleLow;
            }
        } else {
            if (launchAngleLow > MIN_LAUNCH_ANGLE && launchAngleLow < MAX_LAUNCH_ANGLE && launchAngleHigh > MIN_LAUNCH_ANGLE && launchAngleHigh < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = Math.min(launchAngleLow, launchAngleHigh);
            } else if (launchAngleLow > MIN_LAUNCH_ANGLE && launchAngleLow < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = launchAngleLow;
            } else if (launchAngleHigh > MIN_LAUNCH_ANGLE && launchAngleHigh < MAX_LAUNCH_ANGLE) {
                preferredLaunchResult = launchAngleHigh;
            } else {
                preferredLaunchResult = launchAngleLow;
            }
        }

        return preferredLaunchResult;
    }
    public static double calculateVelocity(double flywheelRPM) {
        //return (((flywheelRPM * (2 * Math.PI)) / 60) * (FLYWHEEL_DIAMETER_METERS / 2)) * VELOCITY_TRANSFER_EFFICIENCY;
        return (((Math.PI * FLYWHEEL_DIAMETER_METERS * flywheelRPM) / 60) * VELOCITY_TRANSFER_EFFICIENCY) ;
    }

    public static double calculateTimeOfFlight(double velocity, double angle, double Yi, double Yf) {
        // Convert angle to radians for Math functions
        double radians = Math.toRadians(angle);

        // Initial vertical velocity component
        double Viy = velocity * Math.sin(radians);

        // Using Quadratic Formula: 0 = -0.5 * g * t^2 + vYi * t - deltaY
        // Discriminant: b^2 - 4ac
        double discriminant = Math.pow(Viy, 2) + (2 * (ACCELERATION_DUE_TO_GRAVITY) * (Yf - Yi));

        if (discriminant < 0) {
            // Target height is physically unreachable
            return -1.0;
        }

        return (Viy + Math.sqrt(discriminant)) / ACCELERATION_DUE_TO_GRAVITY;
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
