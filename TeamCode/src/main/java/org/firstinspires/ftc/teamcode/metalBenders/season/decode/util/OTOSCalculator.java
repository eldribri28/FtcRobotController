package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;

public class OTOSCalculator {

    public static OTOSResult getCurrentPosition(SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D pos;
        pos = OTOS.getPosition();
        ROBOT_FIELD_X = pos.x;
        ROBOT_FIELD_Y = pos.y;
        ROBOT_FIELD_H = pos.h;
        return new OTOSResult(ROBOT_FIELD_X, ROBOT_FIELD_Y, ROBOT_FIELD_H);

    }

    public static OTOSResult setCurrentPosition(double x, double y, double h, SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D currentPosition;
        currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
        OTOS.setPosition(currentPosition);
        return new OTOSResult(getCurrentPosition(OTOS).getXPos(), getCurrentPosition(OTOS).getYPos(), getCurrentPosition(OTOS).getHeading());

    }
}
