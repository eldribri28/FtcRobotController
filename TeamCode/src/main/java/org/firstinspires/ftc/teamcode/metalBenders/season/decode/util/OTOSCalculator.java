package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class OTOSCalculator {

    public static OTOSResult getCurrentPosition(SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D pos;
        pos = OTOS.getPosition();
        return new OTOSResult(pos.x, pos.y, pos.h);

    }

    public static OTOSResult setCurrentPosition(double currentX, double currentY, double currentHeading, SparkFunOTOS OTOS) {

        SparkFunOTOS.Pose2D currentPosition;
        currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, currentHeading);
        OTOS.setPosition(currentPosition);
        return new OTOSResult(getCurrentPosition(OTOS).getXPos(), getCurrentPosition(OTOS).getYPos(), getCurrentPosition(OTOS).getHeading());

    }
}
