package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;

@Autonomous(name="Red Autonomous", group="Test")
public class RedTargetAutoOp extends AbstractBaseAutoOp {
    @Override
    protected ColorEnum getTargetColor() {
        return ColorEnum.RED;
    }
}
