package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.AbstractBaseOpMode;

public class IntakeSystem extends AbstractSystem {

    public IntakeSystem(AbstractBaseOpMode baseOpMode) {
        super(baseOpMode);
    }

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_2;
    }

    @Override
    protected void process() {
        if(getBaseOpMode().getHardwareManager().getGamepad().left_trigger > 0) {
            getBaseOpMode().getHardwareManager().getIntakeMotor().setPower(1);
        } else {
            getBaseOpMode().getHardwareManager().getIntakeMotor().setPower(0);
        }
    }
}
