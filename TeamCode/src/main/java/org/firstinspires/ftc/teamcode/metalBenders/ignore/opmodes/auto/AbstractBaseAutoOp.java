package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.auto;

import org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.AbstractBaseOpMode;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.AbstractSystem;

import java.util.Collections;
import java.util.List;

public abstract class AbstractBaseAutoOp extends AbstractBaseOpMode {

    @Override
    protected List<AbstractSystem> getSystems() {
        //TODO create system threads
        return Collections.emptyList();
    }
}
