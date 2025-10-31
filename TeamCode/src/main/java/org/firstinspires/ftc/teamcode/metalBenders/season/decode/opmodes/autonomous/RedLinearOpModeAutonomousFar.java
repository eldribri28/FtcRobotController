package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.StartPositionEnum;

@Autonomous(name="RED FAR Linear Autonomous", group="auto-far", preselectTeleOp = "RED Linear TeleOp")
public class RedLinearOpModeAutonomousFar extends AutonomousBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }

    @Override
    StartPositionEnum getStartPosition() {
        return StartPositionEnum.FAR;
    }
}
