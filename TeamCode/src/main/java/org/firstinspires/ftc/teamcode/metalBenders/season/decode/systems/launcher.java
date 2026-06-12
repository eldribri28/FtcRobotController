package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.HOOD_SERVO_MAX_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.HOOD_SERVO_MIN_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Tuning.MIN_LAUNCH_ANGLE;
import com.qualcomm.robotcore.util.Range;

/**
 * 2025-26 FTC Season DECODE
 * <br/>
 * Class for Robot Launch Controls
 * @author Team 14739 Metal Benders
 * @version 0.1
 * @since 2026-06-08
 */

public class launcher {


    /**
     * <b>setAngle</b> Set Launcher Hood Angle in Degrees.
     */
    public static double setLaunchAngle(double setAngle) {
        double servoMin = HOOD_SERVO_MIN_VALUE;
        double servoMax = HOOD_SERVO_MAX_VALUE;
        if (setAngle == 0) {
            return servoMin;
        } else {
            return Range.clip(((MAX_LAUNCH_ANGLE - setAngle) * (servoMax / ((MAX_LAUNCH_ANGLE - MIN_LAUNCH_ANGLE)))), servoMin, servoMax);
        }
    }

}
