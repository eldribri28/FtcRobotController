package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ACCELERATION_DUE_TO_GRAVITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.FLYWHEEL_DIAMETER_METERS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.VELOCITY_TRANSFER_EFFICIENCY;

public class LaunchCalculator {

    public static LaunchResult calculatePreferredLaunchResult1(double flywheelRPM, double distance) {

        double velocity = calculateVelocity(flywheelRPM);

        boolean solutionViable1 = true;
        boolean solutionViable2 = true;

        // Calculate both angle solutions
        double root = Math.pow(velocity, 4) - ACCELERATION_DUE_TO_GRAVITY * (ACCELERATION_DUE_TO_GRAVITY * distance * distance + 2 * velocity * velocity * (TARGET_HEIGHT - LAUNCH_HEIGHT));
        double launchAngle1 = Math.atan2(((velocity * velocity) - Math.sqrt(root)), (ACCELERATION_DUE_TO_GRAVITY * distance));
        double launchAngle2 = Math.atan2(((velocity * velocity) + Math.sqrt(root)), (ACCELERATION_DUE_TO_GRAVITY * distance));

        // Calculate Max Heights for both solutions
        double maxHeight1 = calculateMaxHeight(velocity, launchAngle1);
        double maxHeight2 = calculateMaxHeight(velocity, launchAngle2);

        double landingAngle1 = calculateLandingAngle(velocity, launchAngle1);
        double landingAngle2 = calculateLandingAngle(velocity, launchAngle2);

        // Solution is Not Viable if launch angle solution is greater than 90 degrees, less than 0 degrees, NaN, or results in a Max Height greater than 1.524m
        if (launchAngle1 > 1.5707 || launchAngle1 < 0 || Double.isNaN(launchAngle1) || maxHeight1 > 2.0) {
            solutionViable1 = false;
        }
        if (launchAngle2 > 1.5707 || launchAngle2 < 0 || Double.isNaN(launchAngle2) || maxHeight2 > 2.0) {
            solutionViable2 = false;
        }
        LaunchResult launchResult1 = new LaunchResult(Math.toDegrees(launchAngle1), Math.toDegrees(landingAngle1), maxHeight1, solutionViable1);
        LaunchResult launchResult2 = new LaunchResult(Math.toDegrees(launchAngle2), Math.toDegrees(landingAngle2), maxHeight2, solutionViable2);
        return determinePreferredLaunchResult(launchResult1, launchResult2);
    }

    private static LaunchResult determinePreferredLaunchResult(LaunchResult launchResult1, LaunchResult launchResult2) {
        LaunchResult preferredLaunchResult = null;
        if (launchResult1.isViable() && launchResult2.isViable()) {
            if (launchResult1.getLandingAngle() > launchResult2.getLandingAngle()) {
                preferredLaunchResult = launchResult1;
            } else {
                preferredLaunchResult = launchResult2;
            }
        } else if (launchResult1.isViable()) {
            preferredLaunchResult = launchResult1;
        } else if (launchResult2.isViable()) {
            preferredLaunchResult = launchResult2;
        }
        return preferredLaunchResult;
    }

    private static double calculateLandingAngle(double velocity, double launchAngle) {
        // Calculate Landing Angles
        double vX = velocity * Math.cos(launchAngle);
        double vY = velocity * Math.sin(launchAngle);
        double tF = ((vY + Math.sqrt(Math.pow(vY, 2) - (2 * ACCELERATION_DUE_TO_GRAVITY * (TARGET_HEIGHT - LAUNCH_HEIGHT)))) / ACCELERATION_DUE_TO_GRAVITY);
        double vF = Math.sqrt(Math.pow(velocity, 2) - (2 * (ACCELERATION_DUE_TO_GRAVITY * (TARGET_HEIGHT - LAUNCH_HEIGHT))));
        //double Vfy = -1 * Math.sqrt(Math.pow(vY, 2) + (2 * (g * (targetHeight - launchHeight))));
        double Vfy = vY - (ACCELERATION_DUE_TO_GRAVITY * tF);
        double Vfx = vX; // We will ignore air resistance
        return Math.atan(Vfy / Vfx);
    }

    public static double calculateTransitTime(double distance, double velocity, double launchAngle) {
        return (distance / (velocity * Math.cos(Math.toRadians(launchAngle)))); // Assuming no air resistance
    }

    private static double calculateMaxHeight(double velocity, double launchAngle) {
        return ((Math.pow(velocity, 2) * (Math.sin(launchAngle) * Math.sin(launchAngle))) / (2 * ACCELERATION_DUE_TO_GRAVITY)) + LAUNCH_HEIGHT;
    }

    public static double calculateVelocity(double flywheelRPM) {
        return (((flywheelRPM * (2 * Math.PI)) / 60) * (FLYWHEEL_DIAMETER_METERS / 2)) * VELOCITY_TRANSFER_EFFICIENCY;
    }

}
