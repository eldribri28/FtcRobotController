package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TARGET_HEIGHT;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.getAngleForFlywheel;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;

public class trajectory {

    /*
    Pre-calculate Look Up Table for trajectory distance results.
     */
    public static double[][] generateTrajectoryLUT() {
        double[][] trajectoryLUT = new double[501][3];
        int index = 0;
        trajectoryLUT[0][0] = 0;
        trajectoryLUT[0][1] = 0;
        trajectoryLUT[0][2] = 0;
        while (index < 500) {
            index++;
            double targetDistance = (index / 100);
            LaunchCalculator.LaunchResult trajectoryResults = LaunchCalculator.getLaunchData(LAUNCH_HEIGHT, TARGET_HEIGHT, targetDistance, 0, 0);
            trajectoryLUT[index][0] = trajectoryResults.getLaunchVelocity();
            trajectoryLUT[index][1] = trajectoryResults.getLaunchAngle();
            trajectoryLUT[index][2] = trajectoryResults.getTOF();
        }
        return trajectoryLUT;
    }

    public static double rpmToEncoderVelocity(double targetRPM) {
        return (((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
    }

    public static double getHoodAngle(double currentFlywheelVelocity, double targetDistance) {
        double flywheelRPM = flywheelVelocityToRpm(currentFlywheelVelocity);
        return getAngleForFlywheel(LAUNCH_HEIGHT, TARGET_HEIGHT, flywheelRPM, targetDistance);
    }

    public static double flywheelVelocityToRpm(double currentFlywheelVelocity) {
        return (currentFlywheelVelocity / 28.0) * 60.0;
    }


}
