package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.tele;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

//@TeleOp(name="Red TeleOp", group="Test")
public class RedTargetTeleOp extends AbstractBaseTeleOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }
}
