package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOSCalculator {

    //private static ;

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
