package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@Disabled
@Autonomous(name="Blue Autonomous", group="Test")
public class BlueTargetAutoOp extends AbstractBaseAutoOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
