package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.DRIVEPID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATEPID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.ROTATION_ACCURACY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.BOT_HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_TIME;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_TIME;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.DriveVelocityResult;

@Autonomous(name="auto drive test")
public class AutonDriveTest extends LinearOpMode {
    private HardwareManager hardwareManager;
    private boolean atTargetPosition = false;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        waitForStart();
        resetRuntime();
        startDrive(0.500, 0.500, 0.0, 0.020);
        telemetry.update();
        while(opModeIsActive()) {
            updateSparkfunOdometry();
            telemetry.addData("atTargetPosition", atTargetPosition);
//            telemetry.update();
        }
    }

    private void updateSparkfunOdometry() {
        SparkFunOTOS.Pose2D pos = hardwareManager.getOtos().getPosition();
        ROBOT_FIELD_X = pos.x;
        ROBOT_FIELD_Y = pos.y;
        ROBOT_FIELD_H = pos.h;
        telemetry.addData("pos.x", pos.x);
        telemetry.addData("pos.y", pos.y);
        telemetry.addData("pos.h", pos.h);

    }

    public void startDrive(double x, double y, double h, double tgtTol) {
        double targetH = AngleUnit.DEGREES.normalize(h);
        if (Math.abs(getDriveDistance(x,y)) < tgtTol) {
            DriveVelocityResult motorVelocities = calculateMotorSpeeds(x, y, h);
            hardwareManager.getLeftFrontMotor().setVelocity(motorVelocities.lfDriveVelocity());
            hardwareManager.getLeftRearMotor().setVelocity(motorVelocities.lrDriveVelocity());
            hardwareManager.getRightFrontMotor().setVelocity(motorVelocities.rfDriveVelocity());
            hardwareManager.getRightRearMotor().setVelocity(motorVelocities.rrDriveVelocity());
            atTargetPosition = false;
        } else {
//            rotateRobot(h);
        }

    }

    private DriveVelocityResult calculateMotorSpeeds(double targetX, double targetY, double targetH) {
        double RX;
        double headingError = AngleUnit.DEGREES.normalize(targetH - Math.cos(normalizedFieldBotHeading() / 180 * Math.PI));
        double x = Math.sin(getDriveAngle(targetX,targetY) / 180 * Math.PI) * drivePidPower(getDriveDistance(targetX,targetY));
        double y = Math.cos(getDriveAngle(targetX,targetY) / 180 * Math.PI) * drivePidPower(getDriveDistance(targetX,targetY));
        if (headingError > 0) {
            RX = rotatePidPower(headingError) * 0.8;
        } else {
            RX = -(rotatePidPower(headingError) * 0.8);
        }
        double cosHeading = Math.cos(normalizedFieldBotHeading() / 180 * Math.PI);
        double sinHeading = Math.sin(normalizedFieldBotHeading() / 180 * Math.PI);
        double rotX = y * sinHeading - x * cosHeading;
        double rotY = y * cosHeading + x * sinHeading;
        rotX = rotX * 1.1;  // Counteract imperfect strafing
        //double Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(RX))), 1));
        double lfDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -((rotY + rotX + RX))); // Denominator));
        double rfDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY - rotX) - RX))); // Denominator));
        double lrDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY - rotX) + RX))); // Denominator));
        double rrDriveVelocity = (int) (AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND * -(((rotY + rotX) - RX))); // Denominator));

        telemetry.addData("lftDriveVelocity", lfDriveVelocity);
        telemetry.addData("rfDriveVelocity", rfDriveVelocity);
        telemetry.addData("lrDriveVelocity", lrDriveVelocity);
        telemetry.addData("rrDriveVelocity", rrDriveVelocity);

        return new DriveVelocityResult(lfDriveVelocity, rfDriveVelocity, lrDriveVelocity, rrDriveVelocity);
    }

    private double getDriveAngle(double targetX, double targetY) {
        double deltaX = ROBOT_FIELD_X - targetX;
        double deltaY = ROBOT_FIELD_Y - targetY;
        telemetry.addData("drive angle", Math.atan2(deltaY, deltaX) / Math.PI * 180);
        return Math.atan2(deltaY, deltaX) / Math.PI * 180;
    }

    private double getDriveDistance(double targetX, double targetY) {
        double deltaX = ROBOT_FIELD_X - targetX;
        double deltaY = ROBOT_FIELD_Y - targetY;
        telemetry.addData("drive distance", Math.abs(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2))));
        return Math.abs(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }

    public void rotateRobot(double h) {
        double steeringCorrection;
        double rotateVelocity;
        double headingError = AngleUnit.DEGREES.normalize(h - Math.cos(normalizedFieldBotHeading()));
        // Keep looping while we are still active and not on heading.
        if (Math.abs(headingError) >= ROTATION_ACCURACY) {
            headingError = AngleUnit.DEGREES.normalize(h - Math.cos(normalizedFieldBotHeading()));
            if (headingError > 0) {
                rotateVelocity = rotatePidPower(headingError) * AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND;
            } else {
                rotateVelocity = -(rotatePidPower(headingError) * AUTON_DRIVE_VELOCITY_TICKS_PER_SECOND);
            }
            // Clip the speed to the maximum permitted value.
            hardwareManager.getLeftFrontMotor().setVelocity(-rotateVelocity);
            hardwareManager.getLeftRearMotor().setVelocity(-rotateVelocity);
            hardwareManager.getRightFrontMotor().setVelocity(rotateVelocity);
            hardwareManager.getRightRearMotor().setVelocity(rotateVelocity);
            atTargetPosition = false;
        } else {
            stopDrive();
            atTargetPosition = true;
        }
    }

    private void stopDrive() {
        hardwareManager.getLeftFrontMotor().setVelocity(0);
        hardwareManager.getLeftRearMotor().setVelocity(0);
        hardwareManager.getRightFrontMotor().setVelocity(0);
        hardwareManager.getRightRearMotor().setVelocity(0);
    }

    private double normalizedFieldBotHeading() {
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        ROBOT_FIELD_H = Double.parseDouble(JavaUtil.formatNumber(AngleUnit.DEGREES.normalize(orientation.getYaw(AngleUnit.DEGREES) + BOT_HEADING_OFFSET), 3));
        telemetry.addData("normalizedFieldBotHeading", ROBOT_FIELD_H);
        return ROBOT_FIELD_H;
    }

    private double rotatePidPower(double h) {
        double headingError = AngleUnit.DEGREES.normalize(h - normalizedFieldBotHeading());
        // Determine the heading current error.
        if (headingError > 180) {
            headingError = headingError - 360;
        } else if (headingError <= -180) {
            headingError = headingError + 360;
        }
        ROTATE_PID_ACCUMULATED_ERROR = (int) (ROTATE_PID_ACCUMULATED_ERROR + headingError);
        if (Math.abs(headingError) < 1) {
            ROTATE_PID_ACCUMULATED_ERROR = 0;
        }
        // Ensure sign of ROTATE_PID_ACCUMULATED_ERROR matches headingError
        if (headingError < 0) {
            ROTATE_PID_ACCUMULATED_ERROR = Math.abs(ROTATE_PID_ACCUMULATED_ERROR) * -1;
        } else {
            ROTATE_PID_ACCUMULATED_ERROR = Math.abs(ROTATE_PID_ACCUMULATED_ERROR) * 1;
        }
        double rotatePID_slope = 0;
        if (ROTATE_PID_LAST_TIME > 0) {
            rotatePID_slope = (int) ((headingError - ROTATE_PID_LAST_ERROR) / (System.currentTimeMillis() - ROTATE_PID_LAST_TIME));
            ROTATE_PID_LAST_TIME = System.currentTimeMillis();
            ROTATE_PID_LAST_ERROR = headingError;
        }
        telemetry.addData("rotatePidPower", Range.clip(Math.abs(Range.clip(headingError, 0, 0.2)) + Range.scale(Math.abs(headingError * ROTATEPID_P + ROTATE_PID_ACCUMULATED_ERROR * ROTATEPID_I + ROTATEPID_D * rotatePID_slope), 0, 180, 0.1, 0.8), 0.2, 1));
        return Range.clip(Math.abs(Range.clip(headingError, 0, 0.2)) + Range.scale(Math.abs(headingError * ROTATEPID_P + ROTATE_PID_ACCUMULATED_ERROR * ROTATEPID_I + ROTATEPID_D * rotatePID_slope), 0, 180, 0.1, 0.8), 0.2, 1);
    }

    private double drivePidPower(double driveDistance) {
        DRIVE_PID_ACCUMULATED_ERROR = (int) (DRIVE_PID_ACCUMULATED_ERROR + (System.currentTimeMillis() - DRIVE_PID_LAST_TIME) * driveDistance);
        if (driveDistance < 1) {
            DRIVE_PID_ACCUMULATED_ERROR = 0;
        }
        double drivePidSlope = 0;
        if (DRIVE_PID_LAST_TIME > 0) {
            drivePidSlope = (int) ((driveDistance - DRIVE_PID_LAST_ERROR) / (System.currentTimeMillis() - DRIVE_PID_LAST_TIME));
            DRIVE_PID_LAST_TIME = System.currentTimeMillis();
            DRIVE_PID_LAST_ERROR = driveDistance;
        }
        telemetry.addData("drivePidPower", Range.scale(Math.abs(driveDistance * DRIVEPID_P + DRIVE_PID_ACCUMULATED_ERROR * DRIVEPID_I + DRIVEPID_D * drivePidSlope), 0, 60, 0.1, 1));
        return Range.scale(Math.abs(driveDistance * DRIVEPID_P + DRIVE_PID_ACCUMULATED_ERROR * DRIVEPID_I + DRIVEPID_D * drivePidSlope), 0, 60, 0.1, 1);
    }
}
