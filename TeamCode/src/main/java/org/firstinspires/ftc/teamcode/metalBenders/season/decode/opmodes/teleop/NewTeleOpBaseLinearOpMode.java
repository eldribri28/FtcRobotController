package org.firstinspires.ftc.teamcode.metalBenders.season.decode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.UNKNOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_DOWN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.LAUNCH_SERVO_UP;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_FAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_LAUNCH_MOTOR_VELOCITY_START;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MANUAL_NEAR_LAUNCH_VELOCITY;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MAX_DRIVE_VELOCITY_TICKS_PER_SECOND;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.Constants.MINIMUM_COLOR_HIT_COUNT_TO_CHANGE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.TurretBearing.getTurretChassisOffset;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.AprilTagEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.IndicatorLedEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.hardware.HardwareManager;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.AutoTargetTurret;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSCalculator;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.util.OTOSResult;

import java.util.Map;


public abstract class NewTeleOpBaseLinearOpMode extends LinearOpMode {
    private HardwareManager hardwareManager;
    private ArtifactColorEnum intakeArtifactColor = UNKNOWN;
    private ArtifactColorEnum launcherArtifactColor = UNKNOWN;
    private AutoTargetTurret autoTargetTurret;
    private boolean isManualLaunchOverrideActive = false;
    private double manualLaunchVelocity = MANUAL_LAUNCH_MOTOR_VELOCITY_START;
    private ArtifactMotifEnum artifactMotifEnum = ArtifactMotifEnum.UNKNOWN;
    abstract AprilTagEnum getTargetAprilTag();
    int launcherPurpleSeenCount = 0;
    int launcherGreenSeenCount = 0;
    int launcherNoColorSeenCount = 0;
    int intakePurpleSeenCount = 0;
    int intakeGreenSeenCount = 0;
    int intakeNoColorSeenCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        Thread autoTargetTurretThread = new Thread(autoTargetTurret);
        try {
            waitForStart();
            resetRuntime();
            autoTargetTurretThread.start();
            autoTargetTurretThread.setPriority(Thread.MAX_PRIORITY);
            hardwareManager.postStartInitialization();
            while (opModeIsActive()) {
                updateTelemetry();
                resetIMU();
                drive();
                intakeOrRejectArtifact();
                clearArtifactFromLaunch();
                setArtifactColors();
                setManualLaunchOverride(autoTargetTurretThread);
                launch();
                telemetry.update();
            }
        } finally {
            autoTargetTurretThread.interrupt();
            autoTargetTurret.teardown();
            telemetry.update();
        }
    }

    private void initialize() {
        hardwareManager = new HardwareManager(hardwareMap, gamepad1, gamepad2);
        autoTargetTurret = new AutoTargetTurret(hardwareManager, getTargetAprilTag());
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
        telemetry.addData("Intake artifact color", intakeArtifactColor.name());
        telemetry.addData("Launcher artifact color", launcherArtifactColor.name());
        telemetry.addData("Turret Limit Switch Left Pressed", hardwareManager.getLimitSwitchLeft().isPressed());
        telemetry.addData("Turret Limit Switch Right Pressed", hardwareManager.getLimitSwitchRight().isPressed());
        telemetry.addData("Turret Angle: (deg)", getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition()));
        OTOSResult otosData = OTOSCalculator.getCurrentPosition(hardwareManager.getOtos());
        telemetry.addLine(String.format("Current Position: %6.1f %6.1f %6.1f  (meter, deg)", otosData.getXPos(), otosData.getYPos(), otosData.getHeading()));

//        telemetry.addData("Target Calculated Heading (deg)", getTargetBearing(otosData.getXPos(), otosData.getYPos()));
        //telemetry.addData("Turret Calculated Heading (deg)", calculateTurretAngleFromOtos(hardwareManager.getTurretMotor().getCurrentPosition(), otosData.getHeading()));

        Map<String, String> aprilTagTelemetry = autoTargetTurret.getTelemetry();
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

        double leftFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftFrontPower;
        double rightFrontVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightFrontPower;
        double leftRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * leftRearPower;
        double rightRearVelocity = MAX_DRIVE_VELOCITY_TICKS_PER_SECOND * rightRearPower;

        telemetry.addData("Bearing", -botHeading);
        telemetry.addData("L-Stick X", -gamepad1.left_stick_x);
        telemetry.addData("L-Stick y", -gamepad1.left_stick_y);
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);

        if(leftFrontVelocity == 0) {
            hardwareManager.getLeftFrontMotor().setPower(0);
        } else {
            hardwareManager.getLeftFrontMotor().setVelocity(leftFrontVelocity);
        }
        if(rightFrontVelocity == 0) {
            hardwareManager.getRightFrontMotor().setPower(0);
        } else {
            hardwareManager.getRightFrontMotor().setVelocity(rightFrontVelocity);
        }
        if(leftRearVelocity == 0) {
            hardwareManager.getLeftRearMotor().setPower(0);
        } else {
            hardwareManager.getLeftRearMotor().setVelocity(leftRearVelocity);
        }
        if(rightRearVelocity == 0) {
            hardwareManager.getRightRearMotor().setPower(0);
        } else {
            hardwareManager.getRightRearMotor().setVelocity(rightRearVelocity);
        }

        telemetry.addData("Commanded Motor (Left Front)", "%.2f", leftFrontVelocity);
        telemetry.addData("Commanded Motor (Right Front)", "%.2f", rightFrontVelocity);
        telemetry.addData("Commanded Motor (Left Rear)", "%.2f", leftRearVelocity);
        telemetry.addData("Commanded Motor (Right Rear)", "%.2f", rightRearVelocity);

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

    private void setArtifactColors() {
        setLauncherArtifactColor();
        setIntakeArtifactColor();
        setArtifactMotifEnum();
    }

    private void setArtifactMotifEnum() {
        //get the motif from the turret if is unknown, may continue to be unknown until turret sees motif tag
        if(artifactMotifEnum == ArtifactMotifEnum.UNKNOWN) {
            artifactMotifEnum = autoTargetTurret.getArtifactMotif();
        }
    }

    private void setIntakeArtifactColor() {
        ArtifactColorEnum intakeColor = getArtifactColor(hardwareManager.getIntakeColorSensor(), "Intake");
        if (intakeColor == ArtifactColorEnum.GREEN) {
            if(intakeGreenSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                intakeArtifactColor = ArtifactColorEnum.GREEN;
                intakeGreenSeenCount = 0;
            } else {
                intakeGreenSeenCount++;
            }
        } else if (intakeColor == ArtifactColorEnum.PURPLE) {
            if(intakePurpleSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                intakeArtifactColor = ArtifactColorEnum.PURPLE;
                intakePurpleSeenCount = 0;
            } else {
                intakePurpleSeenCount++;
            }
        } else {
            if(intakeNoColorSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                intakeArtifactColor = ArtifactColorEnum.NONE;
                intakeGreenSeenCount = 0;
                intakePurpleSeenCount = 0;
            } else {
                intakeNoColorSeenCount++;
            }
        }
    }

    private void setLauncherArtifactColor() {
        ArtifactColorEnum launchColor1 = getArtifactColor(hardwareManager.getLaunchColorSensor(), "Launcher");
        ArtifactColorEnum launchColor2 = getArtifactColor(hardwareManager.getLaunchColorSensor2(), "Launcher2");
        if (launchColor1 == ArtifactColorEnum.GREEN || launchColor2 == ArtifactColorEnum.GREEN) {
            if(launcherGreenSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.GREEN.getLedValue());
                launcherArtifactColor = ArtifactColorEnum.GREEN;
                launcherGreenSeenCount = 0;
            } else {
                launcherGreenSeenCount++;
            }
        } else if (launchColor1 == ArtifactColorEnum.PURPLE || launchColor2 == ArtifactColorEnum.PURPLE) {
            if(launcherPurpleSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.PURPLE.getLedValue());
                launcherArtifactColor = ArtifactColorEnum.PURPLE;
                launcherPurpleSeenCount = 0;
            } else {
                launcherPurpleSeenCount++;
            }
        } else {
            if(launcherNoColorSeenCount >= MINIMUM_COLOR_HIT_COUNT_TO_CHANGE) {
                hardwareManager.getIndicatorLed().setPosition(IndicatorLedEnum.BLACK.getLedValue());
                launcherArtifactColor = ArtifactColorEnum.NONE;
                launcherGreenSeenCount = 0;
                launcherPurpleSeenCount = 0;
            } else {
                launcherNoColorSeenCount++;
            }
        }
    }

    private ArtifactColorEnum getArtifactColor(RevColorSensorV3 colorSensor, String sensorName) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
//        double distanceCM = colorSensor.getDistance(DistanceUnit.CM);

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.2) {
            artifactColorEnum = ArtifactColorEnum.PURPLE;
        } else if (green > red && green > blue && green > 0.2) {
            artifactColorEnum = ArtifactColorEnum.GREEN;
        } else {
            artifactColorEnum = ArtifactColorEnum.NONE;
        }
        telemetry.addData(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        telemetry.addData(sensorName, artifactColorEnum.name());
        return artifactColorEnum;
    }

    private void autoLaunch() {
        if(hardwareManager.getGamepad1().right_trigger > 0 && autoTargetTurret.readyToShoot()) {
            autoLaunchArtifact();
        }
    }

    private void autoLaunchArtifact() {
        hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_UP);
        sleep(75);
        hardwareManager.getLaunchServo().setPosition(LAUNCH_SERVO_DOWN);
    }

    private void clearArtifactFromLaunch() {
        if((isManualLaunchOverrideActive && hardwareManager.getGamepad2().right_bumper)
                || (!isManualLaunchOverrideActive && hardwareManager.getGamepad1().right_bumper)) {
            hardwareManager.getLauncherMotor().setPower(0.4);
            sleep(75);
            autoLaunchArtifact();
        }
    }

    private void autoIntakeArtifact() {
        hardwareManager.getIntakeMotor().setPower(1);
        sleep(25);
        hardwareManager.getIntakeMotor().setPower(0);
    }
//
//    public double getTargetBearing(double x, double y) {
//        double deltaX = 0;
//        double deltaY = 0;
//
//        if (getTargetAprilTag() == AprilTagEnum.BLUE_TARGET) {
//            deltaX = x - (-72);
//            deltaY = y - (-72);
//        } else if (getTargetAprilTag() == AprilTagEnum.RED_TARGET) {
//            deltaX = x - (-72);
//            deltaY = y - (72);
//        }
//        return (Math.toDegrees(Math.atan2(deltaY, deltaX)));// - getTurretChassisOffset(hardwareManager.getTurretMotor().getCurrentPosition() + 360) % 360) ;
//    }
}

