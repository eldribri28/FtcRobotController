package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;

public abstract class AbstractSystem implements Runnable {

    public final AbstractBaseOpMode baseOpMode;
    public abstract SystemPriorityEnum getSystemPriority();

    public AbstractSystem(AbstractBaseOpMode baseOpMode) {
        this.baseOpMode = baseOpMode;
    }

    protected abstract void process();

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            process();
        }
    }

    public AbstractBaseOpMode getBaseOpMode() {
        return baseOpMode;
    }
}
