package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.IMU_YAW;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_ANGLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_IMU_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.IMU_OTOS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CURRENT_ENCODER;


import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Turret Bearing Test", group="sample")
public class TurretBearingTest extends LinearOpMode {

    private HardwareManager hardwareManager;

    private final PIDController turretBearingPid = new PIDController(0.05, 0.0, 0.0);

    private AprilTagEngine aprilTagEngine;

    private AprilTagEnum getTargetAprilTag = AprilTagEnum.BLUE_TARGET;

    private SparkFunOTOS otos;

    private double currentX = 0;
    private double currentY = 0;
    private double currentOtosH = 0;

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

                updateAprilTagFieldPosition();
                getFieldPosition();
                turretRotateLimit();
                moveTurret(calculateTurretError());

                IMU_YAW = hardwareManager.getImu().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                TURRET_CURRENT_ENCODER = hardwareManager.getTurretMotor().getCurrentPosition();
                TURRET_ANGLE = getTurretChassisOffset(TURRET_CURRENT_ENCODER);


                telemetry.addData("IMU Heading", IMU_YAW);
                telemetry.addData("Turret Angle", TURRET_ANGLE);
                telemetry.addData("Turret Offset", TURRET_CHASSIS_OFFSET);
                double currentHeading = OTOSCalculator.getCurrentPosition(otos).getHeading();
                telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", currentX, currentY, currentHeading));


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
        otos = hardwareManager.getOtos();
        aprilTagEngine = new AprilTagEngine(hardwareManager, AprilTagEnum.BLUE_TARGET);
        TURRET_CHASSIS_OFFSET = 0;
        TURRET_LEFT_LIMIT_ENCODER_VALUE = 0;
    }

    private void setFieldPosition(AprilTagDetection targetDetection) {
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            TURRET_CHASSIS_OFFSET = getTurretChassisOffset(TURRET_CURRENT_ENCODER);
            double chassisFieldHeading = (targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
            telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
            telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
            IMU_OTOS_OFFSET = AngleUnit.DEGREES.normalize(IMU_YAW - chassisFieldHeading);
            OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, otos);
        }
    }


    private boolean updateAprilTagFieldPosition() {
        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if (timedDetection != null && timedDetection.getDetection() != null && timedDetection.getAgeInMillis() < AGED_DATA_LIMIT_MILLISECONDS) {
            AprilTagDetection targetDetection = timedDetection.getDetection();
            telemetry.addData("Target detection age(millisecond)", timedDetection.getAgeInMillis());
            setFieldPosition(targetDetection);
            return true;
        } else {
            return false;
        }
    }

    private void getFieldPosition() {
        currentX = OTOSCalculator.getCurrentPosition(otos).getXPos();
        currentY = OTOSCalculator.getCurrentPosition(otos).getYPos();
        currentOtosH = OTOSCalculator.getCurrentPosition(otos).getHeading();
    }


    private void turretRotateLimit() {
        if (hardwareManager.getLimitSwitchRight().isPressed()) {
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER + ( TURRET_TICKS_PER_DEGREE * 90)));
            TURRET_IMU_OFFSET = AngleUnit.DEGREES.normalize(IMU_YAW + getTurretChassisOffset(TURRET_CURRENT_ENCODER));
        } else if (hardwareManager.getLimitSwitchLeft().isPressed()){
            TURRET_LEFT_LIMIT_ENCODER_VALUE = (long) AngleUnit.DEGREES.normalize(Math.round(TURRET_CURRENT_ENCODER - ( TURRET_TICKS_PER_DEGREE * 90)));
            TURRET_IMU_OFFSET = AngleUnit.DEGREES.normalize(IMU_YAW + getTurretChassisOffset(TURRET_CURRENT_ENCODER));
        }
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

    private void moveTurret(double turretError) {
        double setPower = turretBearingPid.calculate(1, turretError);
        //if (canRotateTurret(setPower)) {
        //hardwareManager.getTurretMotor().setPower(setPower);
        //} else {
        //hardwareManager.getTurretMotor().setPower(0);
        //}
        telemetry.addData("Turret Power", setPower);
    }

    private double calculateTurretError() {
        double turretError = 0;
        double chassisError = 0;
        double targetHeading = getTargetBearing(currentX, currentY);
        if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
            turretError = AngleUnit.DEGREES.normalize(  targetHeading - TURRET_CHASSIS_OFFSET );
        }
        telemetry.addData("Turret Error", turretError);
        return turretError;
    }



    public double getTargetBearing(double x, double y) {
        double deltaX = 0;
        double deltaY = 0;

        if (getTargetAprilTag == AprilTagEnum.BLUE_TARGET) {
            deltaX = x - (-1.371);
            deltaY = y - (-1.371);
        } else if (getTargetAprilTag == AprilTagEnum.RED_TARGET) {
            deltaX = x - (-1.371);
            deltaY = y - (1.371);
        }


        return (Math.toDegrees(Math.atan2(deltaY, deltaX)));// - getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition() + 360) % 360) ;
    }



}
