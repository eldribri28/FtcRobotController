package org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums;

public enum IndicatorLedEnum {
    BLACK(0.0),
    RED(0.277),
    ORANGE(0.333),
    YELLOW(0.388),
    GREEN(0.5),
    BLUE(0.611),
    PURPLE(0.722),
    WHITE(1.0);

    private final double value;

    IndicatorLedEnum(double value) {
        this.value = value;
    }

    public double getLedValue() {
        return value;
    }
}