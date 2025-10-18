package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.auto;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.AbstractSystem;

import java.util.Collections;
import java.util.List;

public abstract class AbstractBaseAutoOp extends AbstractBaseOpMode {

    abstract protected ColorEnum getTargetColor();

    @Override
    protected List<AbstractSystem> getSystems() {
        //TODO create system threads
        return Collections.emptyList();
    }
}
