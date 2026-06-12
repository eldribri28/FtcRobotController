package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MIN_LAUNCH_ANGLE;
import com.qualcomm.robotcore.util.Range;

public class launcher {


    public static double setLaunchAngle(double setAngle) {
        double servoMin = 0.30;
        double servoMax = 0.62;
        if (setAngle == 0) {
            return servoMin;
        } else {
            return Range.clip(((MAX_LAUNCH_ANGLE - setAngle) * (servoMax / ((MAX_LAUNCH_ANGLE - MIN_LAUNCH_ANGLE)))), servoMin, servoMax);
        }
    }

}
