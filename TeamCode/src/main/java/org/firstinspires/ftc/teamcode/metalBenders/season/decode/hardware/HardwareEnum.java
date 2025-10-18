package org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware;

public enum HardwareEnum {

    RIGHT_FRONT_MOTOR("RFmotor"),
    LEFT_FRONT_MOTOR("LFmotor"),
    RIGHT_REAR_MOTOR("RRmotor"),
    LEFT_REAR_MOTOR("LRmotor");

    private final String name;

    HardwareEnum(String name){
        this.name = name;
    }

    public String getName() {
        return name;
    }
}
