package org.firstinspires.ftc.teamcode.metalBenders.ignore.sample;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import java.util.List;

@Disabled
@TeleOp(name = "WheelMotorTest")
public class WheelMotorTest extends LinearOpMode {

    private IMU imu;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightRearMotor;
    private DcMotorEx leftRearMotor;
    private Gamepad gamepad;


    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        resetRuntime();
        while (opModeIsActive()) {
            if(gamepad.x){
                rightFrontMotor.setPower(1);
            } else {
                rightFrontMotor.setPower(0);
            }

            if(gamepad.y){
                rightRearMotor.setPower(1);
            } else {
                rightRearMotor.setPower(0);
            }

            if(gamepad.a){
                leftFrontMotor.setPower(1);
            } else {
                leftFrontMotor.setPower(0);
            }

            if(gamepad.b){
                leftRearMotor.setPower(1);
            } else {
                leftRearMotor.setPower(0);
            }
            telemetry.update();
        }
    }

    private void setMotorPower(double rightFrontPower, double leftFrontPower, double rightRearPower, double leftRearPower) {
        rightFrontMotor.setPower(rightFrontPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightRearMotor.setPower(rightRearPower);
        leftRearMotor.setPower(leftRearPower);
    }

    private void initialize() {
        this.gamepad = gamepad1;
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.rightFrontMotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        this.leftFrontMotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        this.rightRearMotor = hardwareMap.get(DcMotorEx.class, "RRmotor");
        this.leftRearMotor = hardwareMap.get(DcMotorEx.class, "LRmotor");
        initializeHardware();
        initializeIMU();
    }

    private void initializeHardware() {
        for(DcMotor motor : List.of(rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor)) {
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }
    }

    private void initializeIMU() {
        ImuOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(DOWN, BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sleep(1000);
    }
}
