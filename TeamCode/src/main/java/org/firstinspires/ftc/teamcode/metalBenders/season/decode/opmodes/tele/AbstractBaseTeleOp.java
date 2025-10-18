package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.AbstractSystem;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.TestSystem;

import java.util.List;

public abstract class AbstractBaseTeleOp extends AbstractBaseOpMode {

    abstract protected ColorEnum getTargetColor();

    @Override
    protected List<AbstractSystem> getSystems() {
        return List.of(new TestSystem(this));
    }
}
