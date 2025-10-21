package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.tele;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

//@TeleOp(name="Blue TeleOp", group="Test")
public class BlueTargetTeleOp extends AbstractBaseTeleOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
