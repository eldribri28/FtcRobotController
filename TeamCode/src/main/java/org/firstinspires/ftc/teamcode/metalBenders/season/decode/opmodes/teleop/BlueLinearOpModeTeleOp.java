package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@TeleOp(name="BLUE Linear TeleOp", group="linear")
public class BlueLinearOpModeTeleOp extends TeleOpBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
