package org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums;

public enum GobildaMotorEnum {
    YELLOWJACKET_30(5281.1),
    YELLOWJACKET_43(3895.9),
    YELLOWJACKET_60(2786.2),
    YELLOWJACKET_84(1993.6),
    YELLOWJACKET_117(1425.1),
    YELLOWJACKET_223(751.8),
    YELLOWJACKET_312(537.7),
    YELLOWJACKET_435(384.5),
    YELLOWJACKET_1150(145.1),
    YELLOWJACKET_1620(103.8),
    YELLOWJACKET_6000(28);

    private double ppr; // Pulses Per Revolution

    GobildaMotorEnum(double ppr) {
        this.ppr = ppr;
    }

    public double getPPR() {
        return ppr;
    }

}
