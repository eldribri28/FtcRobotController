package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

public class LaunchResult {

    double launchAngle1;
    double launchAngle2;
    double landingAngle1;
    double landingAngle2;
    double maxHeight1;
    double maxHeight2;
    double solutionViable1;
    double solutionViable2;



    public LaunchResult(double launchAngle1, double landingAngle1, double maxHeight1, double solutionViable1, double launchAngle2, double landingAngle2, double maxHeight2, double solutionViable2) {

        this.launchAngle1 = launchAngle1;
        this.launchAngle2 = launchAngle2;
        this.landingAngle1 = landingAngle1;
        this.landingAngle2 = landingAngle2;
        this.maxHeight1 = maxHeight1;
        this.maxHeight2 = maxHeight2;
        this.solutionViable1 = solutionViable1;
        this.solutionViable2 = solutionViable2;

    }

    public double getLaunchAngle2() {
        return launchAngle2;
    }

    public double getLandingAngle1() {
        return landingAngle1;
    }

    public double getMaxHeight1() {
        return maxHeight1;
    }

    public double getSolutionViable1() {
        return solutionViable1;
    }

    public double getLaunchAngle1() {
        return launchAngle1;
    }

    public double getLandingAngle2() {
        return landingAngle2;
    }

    public double getMaxHeight2() { return maxHeight2; }

    public double getSolutionViable2() {
        return solutionViable2;
    }

}
