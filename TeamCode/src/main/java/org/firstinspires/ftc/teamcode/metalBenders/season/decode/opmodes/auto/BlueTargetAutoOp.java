package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;

@Autonomous(name="Blue Autonomous", group="Test")
public class BlueTargetAutoOp extends AbstractBaseAutoOp {
    @Override
    protected ColorEnum getTargetColor() {
        return ColorEnum.BLUE;
    }
}
