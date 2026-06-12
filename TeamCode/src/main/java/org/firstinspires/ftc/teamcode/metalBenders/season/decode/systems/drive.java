package org.firstinspires.ftc.teamcode.metalBenders.season.decode.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVE_MOTOR_POWER;

public class drive {

    /**
     * @param gampad1LSY Gamepad 1 Left Stick Y value,
     * @param gamepad1LSX Gamepad 1 Left Stick X value,
     * @param gamepad1RSX Gamepad 1 Right Stick X value,
     * @param gamepad1RB Gamepad 1 Right Bumper value,
     * @param gamepad1RT Gamepad 1 Right Trigger value,
     * @param botHeading Robot Heading from IMU in Radians
     */
    public static DriveMotorPower calculateDrivePower(double gampad1LSY, double gamepad1LSX, double gamepad1RSX, boolean gamepad1RB, boolean gamepad1RT, double botHeading) {
        double y = -gampad1LSY;
        double x = gamepad1LSX * 1.1;
        double rx = gamepad1RSX * 1.0;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //rotX = rotX * 1.1;  // Counteract imperfect strafing

        //calculate power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        double driverSelectedMultiplier = 1.0;
        if(gamepad1RB) {
            driverSelectedMultiplier = 0.8;
        } else if (gamepad1RT) {
            driverSelectedMultiplier = 0.5;
        }
        double leftFrontVelocity = DRIVE_MOTOR_POWER * leftFrontPower * driverSelectedMultiplier;
        double rightFrontVelocity = DRIVE_MOTOR_POWER * rightFrontPower * driverSelectedMultiplier;
        double leftRearVelocity = DRIVE_MOTOR_POWER * leftRearPower * driverSelectedMultiplier;
        double rightRearVelocity = DRIVE_MOTOR_POWER * rightRearPower * driverSelectedMultiplier;

        double LeftFrontMotorPower = 0;
        double RightFrontMotorPower = 0;
        double LeftRearMotorPower = 0;
        double RightRearMotorPower = 0;

        if(leftFrontVelocity != 0) { LeftFrontMotorPower = leftFrontVelocity; }
        if(rightFrontVelocity != 0) { RightFrontMotorPower = rightFrontVelocity; }
        if(leftRearVelocity != 0) { LeftRearMotorPower = leftRearVelocity; }
        if(rightRearVelocity != 0) { RightRearMotorPower = rightRearVelocity; }

        return new DriveMotorPower(RightFrontMotorPower, LeftFrontMotorPower, RightRearMotorPower, LeftRearMotorPower);

    }

    public static class DriveMotorPower {

        private final double RightFrontMotorPower;
        private final double LeftFrontMotorPower;
        private final double RightRearMotorPower;
        private final double LeftRearMotorPower;

        public DriveMotorPower(double RightFrontMotorPower, double LeftFrontMotorPower, double RightRearMotorPower, double LeftRearMotorPower) {
            this.RightFrontMotorPower = RightFrontMotorPower;
            this.LeftFrontMotorPower = LeftFrontMotorPower;
            this.RightRearMotorPower = RightRearMotorPower;
            this.LeftRearMotorPower = LeftRearMotorPower;
        }

        public double getRightFrontMotorPower() { return RightFrontMotorPower; }
        public double getLeftFrontMotorPower() { return LeftFrontMotorPower; }
        public double getRightRearMotorPower() { return RightRearMotorPower; }
        public double getLeftRearMotorPower() { return LeftRearMotorPower; }

        @Override
        public String toString() {
            return "DriveMotorPower{" +
                    "RightFrontMotorPower=" + RightFrontMotorPower +
                    ", LeftFrontMotorPower=" + LeftFrontMotorPower +
                    ", RightRearMotorPower=" + RightRearMotorPower +
                    ", LeftRearMotorPower=" + LeftRearMotorPower +
                    '}';
        }
    }

}
