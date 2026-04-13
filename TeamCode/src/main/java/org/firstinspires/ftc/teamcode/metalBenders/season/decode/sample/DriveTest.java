package org.firstinspires.ftc.teamcode.metalBenders.season.decode.sample;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "DriveTest", group="sample")
public class DriveTest extends LinearOpMode {

    private DcMotorEx rightRearMotor;
    private DcMotorEx leftRearMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftFrontMotor;
    private IMU imu;

    private void setupHardware() {
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        rightRearMotor = hardwareMap.get(DcMotorEx.class, "RRmotor");
        leftRearMotor = hardwareMap.get(DcMotorEx.class, "LRmotor");
        imu =  hardwareMap.get(IMU.class, "imu");
        initializeHardware();
        initializeIMU();
    }

    @Override
    public void runOpMode() {
        setupHardware();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * 0.4;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double rotY = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);


            //calculate power
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1);
            double leftFrontPower = (rotY + rotX + yaw) / denominator;
            double leftRearPower = (rotY - rotX + yaw) / denominator;
            double rightFrontPower = (rotY - rotX - yaw) / denominator;
            double rightRearPower = (rotY + rotX - yaw) / denominator;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftRearPower));
            max = Math.max(max, Math.abs(rightRearPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftRearPower   /= max;
                rightRearPower  /= max;
            }

            leftFrontMotor.setPower(leftFrontPower*1.00);
            rightFrontMotor.setPower(rightFrontPower*1.00);
            leftRearMotor.setPower(leftRearPower*1.00);
            rightRearMotor.setPower(rightRearPower*1.00);

            telemetry.addData("Motor (Left Front)", "%.2f", leftFrontPower);
            telemetry.addData("Motor (Right Front)", "%.2f", rightFrontPower);
            telemetry.addData("Motor (Left Rear)","%.2f", leftRearPower);
            telemetry.addData("Motor (Right Rear)", "%.2f", leftRearPower);
            telemetry.update();
            resetIMU();
        }
    }

    private void resetIMU() {
        if (gamepad1.dpad_up) {
            imu.resetYaw();
        }
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(UP, LEFT);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(1000);
    }

    private void initializeHardware(){
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor)) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for(DcMotor motor : List.of(leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
