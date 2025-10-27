package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;




public class LimelightResult {

    private final int totalTargets;
    private final double targetX;
    private final double targetY;
    private final double targetBearing;
    private final double targetDistance;
    private final String targetColor;

    public LimelightResult(int totalTargets, double targetX, double targetY, double targetBearing, double targetDistance, String targetColor) {

        this.totalTargets = totalTargets;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetBearing = targetBearing;
        this.targetDistance = targetDistance;
        this.targetColor = targetColor;

    }

    public int getTotalTargets(){ return  totalTargets; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetBearing() { return targetBearing; }
    public double getTargetDistance() { return targetDistance; }
    public String getTargetColor() { return targetColor; }

    @Override
    public String toString() {
        return "LimelightResult{" +
                "totalTargets=" + totalTargets +
                ", targetX=" + targetX +
                ", targetY=" + targetY +
                ", targetBearing=" + targetBearing +
                ", targetDistance=" + targetDistance +
                ", targetColor=" + targetColor +
                '}';
    }
}
