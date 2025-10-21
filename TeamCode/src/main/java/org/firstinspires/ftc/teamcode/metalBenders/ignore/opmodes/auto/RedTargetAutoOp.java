package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@Disabled
@Autonomous(name="Red Autonomous", group="Test")
public class RedTargetAutoOp extends AbstractBaseAutoOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.RED_TARGET;
    }
}
