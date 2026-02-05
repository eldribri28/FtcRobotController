package org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums;

public enum AprilTagEnum {
    BLUE_TARGET(20),
    MOTIF_GPP(21),
    MOTIF_PGP(22),
    MOTIF_PPG(23),
    RED_TARGET(24);

    private int id;

    AprilTagEnum(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }

    public static AprilTagEnum findById(int id) {
        for (AprilTagEnum aprilTagEnum : values()) {
            if (aprilTagEnum.id == id) {
                return aprilTagEnum;
            }
        }
        return null;
    }
}
