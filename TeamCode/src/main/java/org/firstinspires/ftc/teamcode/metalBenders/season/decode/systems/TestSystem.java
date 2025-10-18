package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;

public class TestSystem extends AbstractSystem {

    int i = 0;
    public TestSystem(AbstractBaseOpMode baseOpMode) {
        super(baseOpMode);
    }

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_1;
    }

    @Override
    protected void process() {
        getBaseOpMode().addTelemetry("TEST", "TEST" + i);
        i++;
    }
}
