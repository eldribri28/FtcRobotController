package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.auto;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

//@Autonomous(name="Blue Autonomous", group="Test")
public class BlueTargetAutoOp extends AbstractBaseAutoOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
