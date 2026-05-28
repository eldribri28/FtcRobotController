package org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums;

public enum IndicatorLedEnum {
    BLACK(0.0),
    RED(0.280),
    ORANGE(0.326),
    YELLOW(0.391),
    GREEN(0.503),
    BLUE(0.625),
    PURPLE(0.725),
    WHITE(1.0);

    private final double value;

    IndicatorLedEnum(double value) {
        this.value = value;
    }

    public double getLedValue() {
        return value;
    }
}