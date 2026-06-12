package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_H;

public class OTOSCalculator {

    public static OTOSResult getCurrentPosition(SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D pos;
        pos = OTOS.getPosition();
        CAMERA_FIELD_X = pos.x;
        CAMERA_FIELD_Y = pos.y;
        CAMERA_FIELD_H = (pos.h + 360) % 360;
        return new OTOSResult(CAMERA_FIELD_X, CAMERA_FIELD_Y, CAMERA_FIELD_H);

    }

    public static OTOSResult setCurrentPosition(double x, double y, double h, SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D currentPosition;
        currentPosition = new SparkFunOTOS.Pose2D(x, y, ((h + 360) % 360));
        OTOS.setPosition(currentPosition);
        return new OTOSResult(getCurrentPosition(OTOS).getXPos(), getCurrentPosition(OTOS).getYPos(), getCurrentPosition(OTOS).getHeading());

    }
}
