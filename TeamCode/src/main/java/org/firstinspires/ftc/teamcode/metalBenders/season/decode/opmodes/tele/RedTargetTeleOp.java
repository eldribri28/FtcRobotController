package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;

@TeleOp(name="Red TeleOp", group="Test")
public class RedTargetTeleOp extends AbstractBaseTeleOp {
    @Override
    protected ColorEnum getTargetColor() {
        return ColorEnum.RED;
    }
}
