package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.UNKNOWN;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.AGED_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_FAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_START;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_NEAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_AGE_DATA_LIMIT_MILLISECONDS;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_METER_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_P;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_I;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_PID_D;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.TURRET_TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_LEFT_LIMIT_ENCODER_VALUE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.TURRET_CHASSIS_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateTransitTime;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator.calculateVelocity;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AprilTagEngine;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.LaunchResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSResult;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TimedAprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Map;


public abstract class TeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactColorEnum intakeArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor2 = UNKNOWN;
    private final PIDController turretBearingPid = new PIDController(TURRET_PID_P, TURRET_PID_I, TURRET_PID_D);
    private double targetDistance = 0;
    private double launchAngle = 0;
    private double flywheelRPM = 0;
    private double targetRPM = 0;
    private AprilTagEngine aprilTagEngine;
    private boolean isManualLaunchOverrideActive = false;
    private double manualLaunchVelocity = MANUAL_LAUNCH_MOTOR_VELOCITY_START;
    abstract AprilTagEnum getTargetAprilTag();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread aprilTagEngineThread = new Thread(aprilTagEngine);
        try {
            waitForStart();
            resetRuntime();
            aprilTagEngineThread.start();
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            while (opModeIsActive()) {
                updateTelemetry();
                turretRotateLimit();
                resetIMU();
                drive();
                intakeOrRejectArtifact();
                clearArtifactFromLaunch();
                setArtifactColors();
                setManualLaunchOverride(aprilTagEngineThread);
                launch();
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
        aprilTagEngine = new AprilTagEngine(hardwareManager, getTargetAprilTag());
        hardwareManager.getRedLed().setState(false);
        hardwareManager.getGreenLed().setState(false);
    }

    private void updateRuntime() {
        double totalSeconds = getRuntime();
        telemetry.addData(
                "Runtime",
                "%02.0f:%02.0f",
                totalSeconds / 60,
                totalSeconds % 60);
    }

    private void updateTelemetry() {
        updateRuntime();
        telemetry.addData("Manual Launch Controls Active", isManualLaunchOverrideActive);
        telemetry.addData("Target name", getTargetAprilTag().name());
        telemetry.addData("Motif detected", aprilTagEngine.getArtifactMotif().name());
        telemetry.addData("Intake artifact color", intakeArtifactColor.name());
        telemetry.addData("Launcher artifact color", launcherArtifactColor.name());
        telemetry.addData("Launcher artifact color 2", launcherArtifactColor2.name());
        telemetry.addData("Target distance", targetDistance);
        telemetry.addData("Launch Angle", launchAngle);
        telemetry.addData("Turret Limit Switch Left Pressed", hardwareManager.getLimitSwitchLeft().isPressed());
        telemetry.addData("Turret Limit Switch Right Pressed", hardwareManager.getLimitSwitchRight().isPressed());
        telemetry.addData("Turret Angle: (deg)", getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition()));
        OTOSResult otosData = OTOSCalculator.getCurrentPosition(hardwareManager.getOtos());
        telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", otosData.getXPos(), otosData.getYPos(), otosData.getHeading()));

        telemetry.addData("Target Calculated Heading (deg)", getTargetBearing(otosData.getXPos(), otosData.getYPos()));
        //telemetry.addData("Turret Calculated Heading (deg)", calculateTurretAngleFromOtos(hardwareManager.getTurretMotor().getCurrentPosition(), otosData.getHeading()));

        Map<String, String> aprilTagTelemetry = aprilTagEngine.getTelemetry();
        for (String key : aprilTagTelemetry.keySet()) {
            String value = aprilTagTelemetry.get(key);
            telemetry.addData(key, value);
        }
    }

    private void drive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * 0.4;
        YawPitchRollAngles orientation = hardwareManager.getImu().getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        //calculate power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        //double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        //max = Math.max(max, Math.abs(leftRearPower));
        //max = Math.max(max, Math.abs(rightRearPower));

        //if (max > 1.0) {
        //    leftFrontPower /= max;
        //    rightFrontPower /= max;
        //    leftRearPower /= max;
        //    rightRearPower /= max;
        //}

        double leftFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftFrontPower;
        double rightFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightFrontPower;
        double leftRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftRearPower;
        double RightRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightRearPower;

        telemetry.addData("Bearing", -botHeading);
        telemetry.addData("L-Stick X", -gamepad1.left_stick_x);
        telemetry.addData("L-Stick y", -gamepad1.left_stick_y);
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);


        hardwareManager.getLeftFrontMotor().setVelocity(leftFrontVelocity);
        hardwareManager.getRightFrontMotor().setVelocity(rightFrontVelocity);
        hardwareManager.getLeftRearMotor().setVelocity(leftRearVelocity);
        hardwareManager.getRightRearMotor().setVelocity(RightRearVelocity);


        //double voltageCorrectedPower = (DRIVE_MOTOR_MULTIPLIER * 12)  / hardwareMap.voltageSensor.iterator().next().getVoltage();
        //hardwareManager.getLeftFrontMotor().setPower(leftFrontPower * voltageCorrectedPower);
        //hardwareManager.getRightFrontMotor().setPower(rightFrontPower * voltageCorrectedPower);
        //hardwareManager.getLeftRearMotor().setPower(leftRearPower * voltageCorrectedPower);
        //hardwareManager.getRightRearMotor().setPower(rightRearPower * voltageCorrectedPower);

        telemetry.addData("Commanded Motor (Left Front)", "%.2f", leftFrontVelocity);
        telemetry.addData("Commanded Motor (Right Front)", "%.2f", rightFrontVelocity);
        telemetry.addData("Commanded Motor (Left Rear)", "%.2f", leftRearVelocity);
        telemetry.addData("Commanded Motor (Right Rear)", "%.2f", RightRearVelocity);

        telemetry.addData("Actual Motor (Left Front)", "%.2f", hardwareManager.getLeftFrontMotor().getVelocity());
        telemetry.addData("Actual Motor (Right Front)", "%.2f", hardwareManager.getRightFrontMotor().getVelocity());
        telemetry.addData("Actual Motor (Left Rear)", "%.2f", hardwareManager.getLeftRearMotor().getVelocity());
        telemetry.addData("Actual Motor (Right Rear)", "%.2f", hardwareManager.getRightRearMotor().getVelocity());
    }

    private void resetIMU() {
        if (hardwareManager.getGamepad1().dpad_right) {
            hardwareManager.getImu().resetYaw();
        }
    }

    private void intakeOrRejectArtifact() {
        if (hardwareManager.getGamepad1().left_trigger > 0) {
            hardwareManager.getIntakeMotor().setPower(1);
        } else if (hardwareManager.getGamepad1().left_bumper) {
            hardwareManager.getIntakeMotor().setPower(-1);
        } else {
            hardwareManager.getIntakeMotor().setPower(0);
        }
    }

    private void setManualLaunchOverride(Thread aprilTagEngineThread) {
        if(hardwareManager.getGamepad2().dpad_left && hardwareManager.getGamepad2().x) {
            isManualLaunchOverrideActive = true;
            aprilTagEngineThread.interrupt();
            if(getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
                hardwareManager.getTurretMotor().setPower(0.8);
            } else {
                hardwareManager.getTurretMotor().setPower(-0.8);
            }
        } else if (hardwareManager.getGamepad1().dpad_left && hardwareManager.getGamepad1().x) {
            isManualLaunchOverrideActive = false;
            aprilTagEngineThread.start();
        }
    }

    private void launch() {
        if(isManualLaunchOverrideActive) {
            manualLaunch();
        } else {
            autoLaunch();
        }
    }
    private void manualLaunch() {
        if(hardwareManager.getGamepad2().dpad_up) {
            manualLaunchVelocity = MANUAL_FAR_LAUNCH_VELOCITY;
        }
        if(hardwareManager.getGamepad2().dpad_down) {
           manualLaunchVelocity = MANUAL_NEAR_LAUNCH_VELOCITY;
        }
        hardwareManager.getLauncherMotor().setVelocity(manualLaunchVelocity);

        if(hardwareManager.getGamepad2().right_trigger > 0) {
            autoLaunchArtifact();
            autoIntakeArtifact();
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

    private void turretRotateLimit() {
        if(hardwareManager.getLimitSwitchRight().isPressed() || hardwareManager.getLimitSwitchLeft().isPressed()) {
            hardwareManager.getTurretMotor().setPower(0);
            int directionMultiplier = 0;
            if (hardwareManager.getLimitSwitchRight().isPressed()) {
                TURRET_LEFT_LIMIT_ENCODER_VALUE = Math.round(hardwareManager.getTurretMotor().getCurrentPosition() + (TURRET_TICKS_PER_DEGREE * 90));
                directionMultiplier = 1;
            } else if (hardwareManager.getLimitSwitchLeft().isPressed()) {
                TURRET_LEFT_LIMIT_ENCODER_VALUE = Math.round(hardwareManager.getTurretMotor().getCurrentPosition() - (TURRET_TICKS_PER_DEGREE * 90));
                directionMultiplier = -1;
            }
            if(isManualLaunchOverrideActive) {
                hardwareManager.getTurretMotor().setTargetPositionTolerance(5);
                hardwareManager.getTurretMotor().setTargetPosition((int) (TURRET_LEFT_LIMIT_ENCODER_VALUE + Math.round(TURRET_TICKS_PER_DEGREE * 45.0)));
                hardwareManager.getTurretMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareManager.getTurretMotor().setPower(0.8 * directionMultiplier);
            }
        }
    }

    private void setArtifactColors() {
        intakeArtifactColor = getArtifactColor(hardwareManager.getIntakeColorSensor(), "Intake");
        launcherArtifactColor = getArtifactColor(hardwareManager.getLaunchColorSensor(), "Launcher");
        launcherArtifactColor2 = getArtifactColor(hardwareManager.getLaunchColorSensor2(), "Launcher2");
        if (launcherArtifactColor == ArtifactColorEnum.GREEN || launcherArtifactColor2 == ArtifactColorEnum.GREEN) {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
        } else if (launcherArtifactColor == ArtifactColorEnum.PURPLE || launcherArtifactColor2 == ArtifactColorEnum.PURPLE) {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.PURPLE.getLedValue());
        } else {
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.WHITE.getLedValue());
            hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.BLACK.getLedValue());
        }
    }

    private ArtifactColorEnum getArtifactColor(RevColorSensorV3 colorSensor, String sensorName) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        double distanceCM = colorSensor.getDistance(DistanceUnit.CM);

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.2) {
            artifactColorEnum = ArtifactColorEnum.PURPLE;
        } else if (green > red && green > blue && green > 0.2) {
            artifactColorEnum = ArtifactColorEnum.GREEN;
        } else {
            artifactColorEnum = ArtifactColorEnum.NONE;
        }
        telemetry.addData(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        return artifactColorEnum;
    }

    /*
    Target lead calculations to move while shooting.  Assumes infinite acceleration of chassis and no obstacles.
     */
    private double targetLeadCalculation() {

        double transitTime = calculateTransitTime(targetDistance, calculateVelocity(flywheelRPM), launchAngle);
        double angleChange = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (Math.toDegrees(Math.atan2(deltaY, deltaX)) - 90) * 0.5;

        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET && flywheelRPM > 0) {

            double deltaY = targetDistance - (hardwareManager.getGamepad1().left_stick_y * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);
            double deltaX = 0 - (hardwareManager.getGamepad1().left_stick_x * MAX_DRIVE_VELOCITY_METER_PER_SECOND * transitTime);

            angleChange = (-Math.toDegrees(Math.atan2(deltaY, deltaX)) + 90) * 0.5;

        }

        telemetry.addData("Turret Lead (deg)", angleChange);
        return angleChange;

    }

    private void autoLaunch() {
        boolean launchLed = false;
        boolean targetLed = false;
        boolean bearingGood = false;

        TimedAprilTagDetection timedDetection = aprilTagEngine.getTimedTargetDetection();
        if(timedDetection != null && timedDetection.getDetection() != null) {
            targetLed = true;
            AprilTagDetection targetDetection = timedDetection.getDetection();
            long detectionAge = timedDetection.getAgeInMillis();
            telemetry.addData("Target detection age(millisecond)", detectionAge);
            if (detectionAge < AGED_DATA_LIMIT_MILLISECONDS) {
                if (detectionAge < TURRET_AGE_DATA_LIMIT_MILLISECONDS && canRotateTurret(targetDetection.ftcPose.bearing) && targetDetection.id == getTargetAprilTag().getId()) {
                    if (Math.abs(targetDetection.ftcPose.bearing) < 3 ) {
                        bearingGood = true;
                    }
                    telemetry.addData("Turret Angle", (targetDetection.ftcPose.bearing));
                    telemetry.addData("Target ID", targetDetection.id);
                    double setPower = turretBearingPid.calculate(0, targetDetection.ftcPose.bearing);
                    telemetry.addData("Turret Power", setPower);
                    hardwareManager.getTurretMotor().setPower(setPower);

                    if (TURRET_LEFT_LIMIT_ENCODER_VALUE != 0) {
                        TURRET_CHASSIS_OFFSET = getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition());
                        double chassisFieldHeading = (targetDetection.robotPose.getOrientation().getYaw() - TURRET_CHASSIS_OFFSET);
                        telemetry.addData("AprilTag Turret Field Heading (deg)", targetDetection.robotPose.getOrientation().getYaw());
                        telemetry.addData("Calculated Chassis Field Heading (deg)", chassisFieldHeading);
                        OTOSCalculator.setCurrentPosition(targetDetection.robotPose.getPosition().x, targetDetection.robotPose.getPosition().y, chassisFieldHeading, hardwareManager.getOtos());
                    }

                } else {
                    hardwareManager.getTurretMotor().setPower(0);
                }
                targetDistance = targetDetection.ftcPose.range;

                flywheelRPM = (hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0;
                telemetry.addData("flywheel RPM", flywheelRPM);

                LaunchResult launchResult = LaunchCalculator.calculatePreferredLaunchResult1(flywheelRPM, targetDistance);
                if (launchResult != null) {
                    setLaunchAngle(launchResult.getLaunchAngle());
                }

                targetRPM = Math.round(((targetDistance / 1.670) * 920.0) + 1750.0);
                hardwareManager.getLauncherMotor().setVelocity(((targetRPM / 60.0) * 28.0) + ((300.0 / 60.0) * 28.0));
                telemetry.addData("target RPM", targetRPM);

                if (hardwareManager.getGamepad1().right_trigger > 0) {
                    if (Math.abs(((hardwareManager.getLauncherMotor().getVelocity() / 28.0) * 60.0) - targetRPM) < MAX_LAUNCHER_RPM_DIFF_TARGET_TO_ACTUAL && bearingGood) {
                        launchLed = true;
                        autoLaunchArtifact();
                    }
                } else {
                    hardwareManager.getLauncherMotor().setVelocity(0);
                }
            } else {
                //stop rotating turret if detection is too old
                hardwareManager.getTurretMotor().setPower(0);
            }
        } else {
            //stop rotating turret if there is no detection
            hardwareManager.getTurretMotor().setPower(0);
        }
        if (targetLed && !launchLed) {
            hardwareManager.getRedLed().setState(true);
            hardwareManager.getGreenLed().setState(false);
        } else if (launchLed) {
            hardwareManager.getRedLed().setState(true);
            hardwareManager.getGreenLed().setState(false);
        } else {
            hardwareManager.getRedLed().setState(false);
            hardwareManager.getGreenLed().setState(false);
        }
    }

    private void autoLaunchArtifact() {
        //if (isManualLaunchOverrideActive || (launcherArtifactColor != ArtifactColorEnum.NONE && launcherArtifactColor2 != ArtifactColorEnum.NONE)) {
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
            sleep(100);
            hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
            sleep(150);
        //}
    }

    private void clearArtifactFromLaunch() {
        if((isManualLaunchOverrideActive && hardwareManager.getGamepad2().right_bumper)
                || (!isManualLaunchOverrideActive && hardwareManager.getGamepad1().right_bumper)) {
            hardwareManager.getLauncherMotor().setPower(0.4);
            sleep(100);
            autoLaunchArtifact();
        }
    }

    private void autoIntakeArtifact() {
        hardwareManager.getIntakeMotor().setPower(1);
        sleep(25);
        hardwareManager.getIntakeMotor().setPower(0);
    }

    private void setLaunchAngle(double launchAngle) {
        double positionValue = Math.abs(((70.0 - launchAngle) / 35.0));
        if(positionValue >= 0 && positionValue <= 0.8) {
            hardwareManager.getAngleServo().setPosition(positionValue);
        }
        this.launchAngle = launchAngle;
    }

    public double getTargetBearing(double x, double y) {
        double deltaX = 0;
        double deltaY = 0;

        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
            deltaX = x - (-72);
            deltaY = y - (-72);
        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET) {
            deltaX = x - (-72);
            deltaY = y - (72);
        }
        return (Math.toDegrees(Math.atan2(deltaY, deltaX)));// - getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition() + 360) % 360) ;
    }
}

