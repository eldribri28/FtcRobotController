package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;

@Disabled
@TeleOp(name="Blue TeleOp", group="Test")
public class BlueTargetTeleOp extends AbstractBaseTeleOp {

    @Override
    protected AprilTagEnum getTargetAprilTag() {
        return AprilTagEnum.BLUE_TARGET;
    }
}
