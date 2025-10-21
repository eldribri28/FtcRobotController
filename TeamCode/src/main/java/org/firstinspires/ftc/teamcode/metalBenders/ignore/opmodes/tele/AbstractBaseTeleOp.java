package org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.tele;

import org.firstinspires.ftc.teamcode.metalBenders.ignore.opmodes.AbstractBaseOpMode;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.AbstractSystem;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.ManualIntakeSystem;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.ManualDriveSystem;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.AutoTargetingLaunchSystem;

import java.util.List;

public abstract class AbstractBaseTeleOp extends AbstractBaseOpMode {

    @Override
    protected List<AbstractSystem> getSystems() {
        return List.of(
                new ManualDriveSystem(),
                new ManualIntakeSystem(),
                new AutoTargetingLaunchSystem(getTargetAprilTag()));
    }
}
