package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

public enum SystemPriorityEnum {
    PRIORITY_1(1),
    PRIORITY_2(2),
    PRIORITY_3(3),
    PRIORITY_4(4),
    PRIORITY_5(5),
    PRIORITY_6(6),
    PRIORITY_7(7),
    PRIORITY_8(8),
    PRIORITY_9(9),
    PRIORITY_10(10);

    private final int value;

    SystemPriorityEnum(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
