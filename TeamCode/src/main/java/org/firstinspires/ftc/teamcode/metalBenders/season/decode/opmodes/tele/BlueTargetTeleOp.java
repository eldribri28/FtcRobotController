package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;

@TeleOp(name="Blue TeleOp", group="Test")
public class BlueTargetTeleOp extends AbstractBaseTeleOp {
    @Override
    protected ColorEnum getTargetColor() {
        return ColorEnum.BLUE;
    }
}
