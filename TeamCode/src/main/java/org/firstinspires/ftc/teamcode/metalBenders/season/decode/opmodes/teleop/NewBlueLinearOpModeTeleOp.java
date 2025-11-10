package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@TeleOp(name="NEW BLUE Linear TeleOp", group="linear")
public class NewBlueLinearOpModeTeleOp extends NewTeleOpBaseLinearOpMode {

    @Override
    AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
