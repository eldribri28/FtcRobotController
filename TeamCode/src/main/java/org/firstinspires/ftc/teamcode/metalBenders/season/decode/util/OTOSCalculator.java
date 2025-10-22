package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OTOSCalculator {

    private static SparkFunOTOS OTOS;

    public static OTOSResult getCurrentPosition() {

        SparkFunOTOS.Pose2D pos;
        pos = OTOS.getPosition();
        return new OTOSResult(pos.x, pos.y, pos.h);
    }

    public static OTOSResult setCurrentPosition(double currentX, double currentY, double currentHeading) {
        SparkFunOTOS.Pose2D currentPosition;
        currentPosition = new SparkFunOTOS.Pose2D(currentX, currentY, currentHeading);
        OTOS.setPosition(currentPosition);
        return new OTOSResult(getCurrentPosition().getXPos(), getCurrentPosition().getYPos(), getCurrentPosition().getHeading());
    }

    /**
     * Configures the SparkFun OTOS.
     */
    private void initOTOS() {

        OTOS = hardwareMap.get(SparkFunOTOS.class, "SparkFunOTOS");

        SparkFunOTOS.Pose2D offset;

        OTOS.setLinearUnit(DistanceUnit.METER);
        OTOS.setAngularUnit(AngleUnit.DEGREES);
        offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        OTOS.setOffset(offset);
        OTOS.setLinearScalar(1);
        OTOS.setAngularScalar(1);
        OTOS.calibrateImu();
        OTOS.resetTracking();
    }


}
