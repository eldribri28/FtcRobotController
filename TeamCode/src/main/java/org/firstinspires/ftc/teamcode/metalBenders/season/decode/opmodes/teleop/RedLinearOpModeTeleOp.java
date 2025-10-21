package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@TeleOp(name="RED Linear TeleOp", group="linear")
public class RedLinearOpModeTeleOp extends TeleOpBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }
}
