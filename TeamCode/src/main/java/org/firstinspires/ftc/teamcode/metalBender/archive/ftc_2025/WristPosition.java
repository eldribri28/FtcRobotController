package org.firstinspires.ftc.teamcode.metalBender.archive.ftc_2025;

public enum WristPosition {
    UP(.95),
    DOWN(0.1),
    MIDDLE(0.4);

    private final double value;

    WristPosition(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}