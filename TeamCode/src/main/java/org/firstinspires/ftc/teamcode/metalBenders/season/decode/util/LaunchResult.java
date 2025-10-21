package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

public class LaunchResult {

    private final double launchAngle;
    private final double landingAngle;
    private final double maxHeight;
    private final boolean viable;

    public LaunchResult(double launchAngle, double landingAngle, double maxHeight, boolean viable) {
        this.launchAngle = launchAngle;
        this.landingAngle = landingAngle;
        this.maxHeight = maxHeight;
        this.viable = viable;
    }

    public double getLaunchAngle() {
        return launchAngle;
    }

    public double getLandingAngle() {
        return landingAngle;
    }

    public double getMaxHeight() {
        return maxHeight;
    }

    public boolean isViable() {
        return viable;
    }

    @Override
    public String toString() {
        return "LaunchResult{" +
                "launchAngle=" + launchAngle +
                ", landingAngle=" + landingAngle +
                ", maxHeight=" + maxHeight +
                ", viable=" + viable +
                '}';
    }
}
