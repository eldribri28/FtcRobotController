package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

public class LaunchCalculator {

    public static LaunchResult CalculateShot(double targetDistance, double launchHeight, double targetHeight, double flywheelDiameterMeters, double flywheelRPM, double velocityTransferEfficiency) {

            // Input Variables
            //double targetDistance = 2.7;
            //double launchHeight = 0.3;
            //double targetHeight = 1.175;
            double g = 9.81;
            //double flywheelDiameterMeters = 0.150;
            //double flywheelRPM = 2000;
            //double velocityTransferEfficiency = 0.43;

            // Calculate angular velocity from RPM
            double velocity = (((flywheelRPM * (2 * Math.PI)) / 60) * (flywheelDiameterMeters / 2)) * velocityTransferEfficiency;

            // Calculate the launch angle(s) with solutions that don't exceed the max height of 1.524m
            LaunchResult launchResults = calculateLaunchAngle(velocity, targetDistance, launchHeight, targetHeight, g);
            double launchAngle = Math.toDegrees(launchResults.getLaunchAngle());
            double landingAngle = Math.toDegrees(launchResults.getLandingAngle());
            double maxHeight = launchResults.getMaxHeight();
            double launchAngle1 = Math.toDegrees(launchResults.getLaunchAngle1());
            double landingAngle1 = Math.toDegrees(launchResults.getLandingAngle1());
            double maxHeight1 = launchResults.getMaxHeight1();
            double solutionViable1 = launchResults.getSolutionViable1();
            double launchAngle2 = Math.toDegrees(launchResults.getLaunchAngle2());
            double landingAngle2 = Math.toDegrees(launchResults.getLandingAngle2());
            double maxHeight2 = launchResults.getMaxHeight2();
            double solutionViable2 = launchResults.getSolutionViable2();

            if (solutionViable1 == 1 && solutionViable2 == 0) {
                launchAngle = launchAngle1;
                landingAngle = landingAngle1;
                maxHeight = maxHeight1;
            } else if (solutionViable1 == 0 && solutionViable2 == 1) {
                launchAngle = launchAngle2;
                landingAngle = landingAngle2;
                maxHeight = maxHeight2;
            } else if (Math.abs(landingAngle1) > Math.abs(landingAngle2)) {
                launchAngle = launchAngle1;
                landingAngle = landingAngle1;
                maxHeight = maxHeight1;
            } else if (Math.abs(landingAngle1) < Math.abs(landingAngle2)) {
                launchAngle = launchAngle2;
                landingAngle = landingAngle2;
                maxHeight = maxHeight2;
            }


            // Output Results
            //telemetry.addData("Launch Solution 1", ' ');
            //telemetry.addData("Launch Angle", launchAngle1);
            //telemetry.addData("Landing Angle", landingAngle1);
            //telemetry.addData("Max Height", maxHeight1);
            //telemetry.addData("Solution Viable", solutionViable1);
            //telemetry.addLine();
            //telemetry.addData("Launch Solution 2",' ');
            //telemetry.addData("Launch Angle 2", launchAngle2);
            //telemetry.addData("Landing Angle 2", landingAngle2);
            //telemetry.addData("Max Height", maxHeight2);
            //telemetry.addData("Solution Viable", solutionViable2);
            //telemetry.addLine();
            //telemetry.addData("Launcher Data", ' ');
            //telemetry.addData("Launch Velocity", velocity);
            //telemetry.update();

            return new LaunchResult(launchAngle1, landingAngle1, maxHeight1, solutionViable1, launchAngle2, landingAngle2, maxHeight2, solutionViable2, launchAngle, landingAngle, maxHeight);

    }

    public static LaunchResult calculateLaunchAngle(double velocity, double distance, double launchHeight, double targetHeight, double g) {

        double solutionViable1 = 1;
        double solutionViable2 = 1;

        // Calculate both angle solutions
        double root = Math.pow(velocity, 4) - g * (g * distance * distance + 2 * velocity * velocity * (targetHeight - launchHeight));
        double launchAngle1 = Math.atan2( ( (velocity * velocity) - Math.sqrt(root) ), (g * distance) );
        double launchAngle2 = Math.atan2( ( (velocity * velocity) + Math.sqrt(root) ), (g * distance) );

        // Calculate Max Heights for both solutions
        double maxHeight1 = calculateMaxHeight(velocity, launchAngle1, launchHeight, g);
        double maxHeight2 = calculateMaxHeight(velocity, launchAngle2, launchHeight, g);

        double landingAngle1 = calculateLandingAngle(velocity, launchAngle1, launchHeight, targetHeight, g);
        double landingAngle2 = calculateLandingAngle(velocity, launchAngle2, launchHeight, targetHeight, g);

        // Solution is Not Viable if launch angle solution is greater than 90 degrees, less than 0 degrees, NaN, or results in a Max Height greater than 1.524m
        if (launchAngle1 > 1.5707 || launchAngle1 < 0 || Double.isNaN(launchAngle1) || maxHeight1 > 1.524) {
            solutionViable1 = 0;
        }
        if (launchAngle2 > 1.5707 || launchAngle2 < 0 || Double.isNaN(launchAngle2) || maxHeight2 > 1.524) {
            solutionViable2 = 0;
        }

        return new LaunchResult(launchAngle1, landingAngle1, maxHeight1, solutionViable1, launchAngle2, landingAngle2, maxHeight2, solutionViable2, 0, 0, 0);

    }

    public static double calculateLandingAngle(double velocity, double launchAngle, double launchHeight, double targetHeight, double g) {

        // Calculate Landing Angles
        double Vx = velocity * Math.cos(launchAngle);
        double Vy = velocity * Math.sin(launchAngle);
        double Tf = ((Vy + Math.sqrt(Math.pow(Vy, 2) - (2 * g * (targetHeight - launchHeight)))) / g);
        double Vf = Math.sqrt(Math.pow(velocity, 2) - (2 * (g * (targetHeight - launchHeight))));
        //double Vfy = -1 * Math.sqrt(Math.pow(Vy, 2) + (2 * (g * (targetHeight - launchHeight))));
        double Vfy = Vy - (g * Tf);
        double Vfx = Vx; // We will ignore air resistance

        return Math.atan(Vfy / Vfx);

    }

    public static double calculateMaxHeight(double velocity, double launchAngle, double launchHeight, double g) {

        double maxHeight = ((Math.pow(velocity, 2) * (Math.sin(launchAngle) * Math.sin(launchAngle))) / (2 * g)) + launchHeight;

        return maxHeight;

    }

}
