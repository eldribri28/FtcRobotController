package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.auto;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;


//@Autonomous(name="Red Autonomous", group="Test")
public class RedTargetAutoOp extends AbstractBaseAutoOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }
}
