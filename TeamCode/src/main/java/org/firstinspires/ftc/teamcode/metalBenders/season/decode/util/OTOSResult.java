package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

public class OTOSResult {

    private final double xPos;
    private final double yPos;
    private final double heading;


    public OTOSResult(double xPos, double yPos, double heading) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.heading = heading;
    }

    public double getXPos() {
        return xPos;
    }
    public double getYPos() {
        return yPos;
    }
    public double getHeading() {
        return heading;
    }

    @Override
    public String toString() {
        return "OTOSResult{" +
                "xPos=" + xPos +
                ", yPos=" + yPos +
                ", heading=" + heading +
                '}';
    }
}
