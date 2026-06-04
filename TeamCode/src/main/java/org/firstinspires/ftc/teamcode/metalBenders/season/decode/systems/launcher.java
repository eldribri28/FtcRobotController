package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCH_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MIN_LAUNCH_ANGLE;
import com.qualcomm.robotcore.util.Range;

public class launcher {





    public static double setLaunchAngle(double setAngle) {
        return Range.clip(((MAX_LAUNCH_ANGLE - setAngle) * (0.65/((MAX_LAUNCH_ANGLE-MIN_LAUNCH_ANGLE)))), 0.0, 0.65);
    }


}
