package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

public class DriveVelocityResult {

    double lfDriveVelocity;
    double rfDriveVelocity;
    double lrDriveVelocity;
    double rrDriveVelocity;

    public DriveVelocityResult(double lfDriveVelocity, double rfDriveVelocity, double lrDriveVelocity, double rrDriveVelocity) {
        this.lfDriveVelocity = lfDriveVelocity;
        this.rfDriveVelocity = rfDriveVelocity;
        this.lrDriveVelocity = lrDriveVelocity;
        this.rrDriveVelocity = rrDriveVelocity;
    }

    public double lfDriveVelocity() {
        return lfDriveVelocity;
    }
    public double rfDriveVelocity() {
        return rfDriveVelocity;
    }
    public double lrDriveVelocity() {
        return lrDriveVelocity;
    }
    public double rrDriveVelocity() {
        return rrDriveVelocity;
    }
}
