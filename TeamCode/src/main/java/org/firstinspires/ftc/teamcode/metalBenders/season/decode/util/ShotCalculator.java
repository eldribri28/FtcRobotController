package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_LAST_TIMESTAMP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_LAST_YAW;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_LAST_DISTANCE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_YAW_RATE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_TARGET_CLOSE_RATE;


/**
 * 2025-26 FTC Season DECODE
 * <br/>
 * Class for calculating artifact ToF and Shoot on the Move
 *
 * @author Team 14739 Metal Benders
 * @version 0.1
 * @since 2026-04-30
 */
public class ShotCalculator {

    /**
     * <b>robotX</b> RobotPose X field coordinates
     * <b>robotY</b> RobotPose X field coordinates
     * <b>targetBearing</b> camera bearing to aprilTag
     * <b>targetDistance</b> distance of aprilTag from camera
     */
    public static void updateTargetDiff(double targetYaw, double targetDistance) {

        double currentTimestamp = System.currentTimeMillis();
        double dt = currentTimestamp - ROBOT_LAST_TIMESTAMP;

        double ROBOT_TARGET_YAW_RATE;
        double ROBOT_TARGET_CLOSE_RATE;
        if (ROBOT_LAST_TIMESTAMP == 0 || dt > 100) {
            ROBOT_LAST_YAW = targetYaw;
            ROBOT_LAST_DISTANCE = targetDistance;
            ROBOT_TARGET_YAW_RATE = 0;
            ROBOT_TARGET_CLOSE_RATE = 0;
        } else {
            ROBOT_TARGET_YAW_RATE = ((targetYaw - ROBOT_LAST_YAW) / dt) / 1000; // Yaw change rate in DEGREES/SEC
            ROBOT_TARGET_CLOSE_RATE = ((targetDistance - ROBOT_LAST_DISTANCE) / dt) / 1000; // Target close rate in M/S
            ROBOT_LAST_YAW = targetYaw;
            ROBOT_LAST_DISTANCE = targetDistance;
        }

    }

    public static double calculateLeadAngle(double TOF) {
        return ROBOT_TARGET_YAW_RATE * TOF;
    }

}
