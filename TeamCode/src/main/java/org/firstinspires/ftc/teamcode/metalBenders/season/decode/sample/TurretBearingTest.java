package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CURRENT_ENCODER;


import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Turret Bearing Test", group="sample")
public class TurretBearingTest extends LinearOpMode {

    private HardwareManager hardwareManager;

    private final PIDController turretBearingPid = new PIDController(0.025, 0.00, 0.01);

    private AprilTagEngine aprilTagEngine;

    private AprilTagEnum getTargetAprilTag = AprilTagEnum.RED_TARGET;

    private SparkFunOTOS otos;

    private double currentX = 0;
    private double currentY = 0;
    private double currentOtosH = 0;
    private double aprilTagBearing = 0;
    private double IMU_OTOS_OFFSET;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);

        try {
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            while (opModeIsActive()) {

                TURRET_CURRENT_ENCODER = hardwareManager.getTurretMotor().getCurrentPosition();
                TURRET_ANGLE = getTurretChassisOffset(TURRET_CURRENT_ENCODER);

                turretRotateLimit();

                if (updateAprilTagFieldPosition()) {
                    moveTurret(aprilTagBearing);
                } else {
                    moveTurret(calculateTurretError());
                }
                getFieldPosition();

                telemetry.addData("Turret Angle", TURRET_ANGLE);
                telemetry.addData("Turret Offset", TURRET_CHASSIS_OFFSET);
                telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", currentX, currentY, currentOtosH));

                telemetry.update();

            }
        } finally {
            aprilTagEngineThread.interrupt();
            aprilTagEngine.teardown();
            telemetry.update();
        }

    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //otos = hardwareManager.getOtos();
        //SparkFunOTOS.Pose2D offset;
        //otos.setLinearUnit(DistanceUnit.METER);
        //otos.setAngularUnit(AngleUnit.DEGREES);
        //offset = new SparkFunOTOS.Pose2D(0, 0, 90);
        //otos.setOffset(offset);
        aprilTagEngine = new AprilTagEngine(hardwareManager, AprilTagEnum.RED_TARGET);
        TURRET_CHASSIS_OFFSET = 0;
        TURRET_LEFT_LIMIT_ENCODER_VALUE = 0;
    }

    private boolean canRotateTurret(double input) {
        if (input < 0 && hardwareManager.getLimitSwitchRight().isPressed()) {
            return false;
        } else if (input > 0 && hardwareManager.getLimitSwitchLeft().isPressed()){
            return false;
        } else {
            return true;
        }
    }

    private void turretRotateLimit() {
        if (hardwareManager.getLimitSwitchRight().isPressed()) {
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER + ( TURRET_TICKS_PER_DEGREE * 90)));
        } else if (hardwareManager.getLimitSwitchLeft().isPressed()){
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER - ( TURRET_TICKS_PER_DEGREE * 90)));
        }
    }

    /*
    Set the field position based on apriltag
     */
    private void setFieldPosition(AprilTagDetection targetDetection) {
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
            TURRET_CHASSIS_OFFSET = getTurretChassisOffset(TURRET_CURRENT_ENCODER);
            double chassisFieldHeading = AngleUnit.DEGREES.normalize(targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
            IMU_OTOS_OFFSET = AngleUnit.DEGREES.normalize(chassisFieldHeading - orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
            telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
            OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, otos);
        }
    }

    /*
    If a tag is in view, update the robot position
     */
    private boolean updateAprilTagFieldPosition() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if (timedDetection != null && timedDetection.getDetection() != null && timedDetection.getAgeInMillis() < AGED_DATA_LIMIT_MILLISECONDS) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            telemetry.addData("Target detection age(millisecond)", timedDetection.getAgeInMillis());
            setFieldPosition(targetDetection);
            aprilTagBearing = targetDetection.ftcPose.bearing;
            return true;
        } else {
            return false;
        }
    }
    /*
    Lookup current field positon and heading
     */
    private void getFieldPosition() {
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        currentX = OTOSCalculator.getCurrentPosition(otos).getXPos();
        currentY = OTOSCalculator.getCurrentPosition(otos).getYPos();
        currentOtosH = AngleUnit.DEGREES.normalize(IMU_OTOS_OFFSET + orientation.getYaw(AngleUnit.DEGREES));
        //currentOtosH = OTOSCalculator.getCurrentPosition(otos).getHeading();
    }

    /*
    Move the turret to the target
     */
    private void moveTurret(double turretError) {
        telemetry.addData("Turret Error", turretError);
        double setPower = turretBearingPid.calculate(0, turretError);
        setPower = Range.clip(setPower, -1, 1);
        if (canRotateTurret(setPower) && Math.abs(turretError) > 1) {
            hardwareManager.getTurretMotor().setPower(setPower);
        } else {
            hardwareManager.getTurretMotor().setVelocity(0);
        }
        telemetry.addData("Turret Power", setPower);
    }

    /*
    Calculate the current turret error if the camera doesn't see a tag
     */
    private double calculateTurretError() {
        double turretError = 0;
        double turretHeading = 0;
        double chassisError = 0;
        double targetHeading = getTargetBearing(currentX, currentY);
        telemetry.addData("Target Heading", targetHeading);
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            turretHeading = AngleUnit.DEGREES.normalize(currentOtosH + TURRET_ANGLE );
            telemetry.addData("Turret - Heading", turretError);
            turretError = AngleUnit.DEGREES.normalize( turretHeading - targetHeading );
            telemetry.addData("Turret - Heading - Target Heading", turretError);
        }
        return -turretError;
    }


    /*
    Locate target bearing based on robot position
     */
    public double getTargetBearing(double x, double y) {
        double deltaX = 0;
        double deltaY = 0;

        if (getTargetAprilTag == AprilTagEnum.BLUE_TARGET) {
            deltaX = (-1.371) - x;
            deltaY = (-1.371) - y;
        } else if (getTargetAprilTag == AprilTagEnum.RED_TARGET) {
            deltaX = (-1.371) - x;
            deltaY = (1.371) - y;
        }

        return (AngleUnit.DEGREES.normalize(Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90));// - getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition() + 360) % 360) ;
    }



}
