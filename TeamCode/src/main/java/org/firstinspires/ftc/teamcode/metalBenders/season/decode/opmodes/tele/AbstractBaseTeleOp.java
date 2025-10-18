package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.tele;

import org.firstinspires.ftc.teamcode.metalBenders.config.ColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.AbstractSystem;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems.ManualDriveSystem;

import java.util.List;

public abstract class AbstractBaseTeleOp extends AbstractBaseOpMode {

    abstract protected ColorEnum getTargetColor();

    @Override
    protected List<AbstractSystem> getSystems() {
        return List.of(
                new ManualDriveSystem(this),
                new IntakeSystem(this));
    }
}
