package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentricDriveMode (Blocks to Java)")
public class FieldCentricDriveMode extends LinearOpMode {

  private CRServo Grip1;
  private Servo Wrist;
  private Servo Ramp;
  private DcMotor LiftMotor1;
  private DcMotor LiftMotor2;
  private DcMotor ExtendArm;
  private DcMotor RaiseArm;
  private DcMotor LFmotor;
  private DcMotor LRmotor;
  private DcMotor RFmotor;
  private DcMotor RRmotor;
  private BNO055IMU imu;
  private TouchSensor ArmLimit;

  int ArmExtension;
  double ArmExtensionInches;

  /**
   * Describe this function...
   */
  private void scoop_in() {
    Grip1.setPwmEnable();
    if (gamepad2.right_trigger == 1) {
      Grip1.setDirection(CRServo.Direction.REVERSE);
      Grip1.setPower(1);
    } else if (gamepad2.left_trigger == 1) {
      Grip1.setDirection(CRServo.Direction.FORWARD);
      Grip1.setPower(0.3);
    } else {
      Grip1.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void ResetIMU() {
    if (gamepad1.dpad_up) {
      initializeIMU();
    }
  }

  /**
   * Describe this function...
   */
  private void Wrist_Middle() {
    if (gamepad2.x) {
      Wrist.setPosition(0.4);
    }
  }

  /**
   * Describe this function...
   */
  private void Wrist_Down() {
    if (gamepad2.b) {
      Wrist.setPosition(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Ramp2() {
    if (!(gamepad2.right_trigger == 0)) {
      Ramp.setPosition(0);
    } else {
      Ramp.setPosition(1);
    }
  }

  /**
   * Describe this function...
   */
  private void Wrist_Up() {
    // AS5600 Encoder Angle In Degrees
    if (gamepad2.a) {
      Wrist.setPosition(1);
    } else if (AS5600Block.AngleInDeg("AS5600sensor") < 20) {
      Wrist.setPosition(1);
    } else {
    }
  }

  /**
   * Describe this function...
   */
  private void LiftRobot() {
    if (gamepad1.b) {
      LiftMotor1.setPower(1);
      LiftMotor2.setPower(1);
    } else if (gamepad1.a) {
      LiftMotor1.setPower(-1);
      LiftMotor2.setPower(-1);
    } else {
      LiftMotor1.setPower(0);
      LiftMotor2.setPower(0);
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int Arm90DegOffset;
    double ArmTicksPerInch;
    double ArmPivotToGround;
    int ArmPivotToGripperCenter;
    boolean BucketDown;
    float Y;
    float X;
    double RX;
    double botHeading;
    double cosHeading;
    double sinHeading;
    double rotX;
    double rotY;
    double Denominator;
    double Multiplier;

    Grip1 = hardwareMap.get(CRServo.class, "Grip1");
    Wrist = hardwareMap.get(Servo.class, "Wrist");
    Ramp = hardwareMap.get(Servo.class, "Ramp");
    LiftMotor1 = hardwareMap.get(DcMotor.class, "LiftMotor1");
    LiftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
    ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
    RaiseArm = hardwareMap.get(DcMotor.class, "Raise Arm");
    LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
    LRmotor = hardwareMap.get(DcMotor.class, "LRmotor");
    RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    ArmLimit = hardwareMap.get(TouchSensor.class, "ArmLimit");

    initializeIMU();
    Arm90DegOffset = 75;
    ArmTicksPerInch = 82.5;
    ArmPivotToGround = 9.625;
    ArmPivotToGripperCenter = 12;
    ArmExtensionInches = -(ArmExtension / ArmTicksPerInch);
    ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ExtendArm.setDirection(DcMotor.Direction.FORWARD);
    ExtendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RaiseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RaiseArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RaiseArm.setDirection(DcMotor.Direction.FORWARD);
    LiftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LFmotor.setDirection(DcMotor.Direction.REVERSE);
    LRmotor.setDirection(DcMotor.Direction.REVERSE);
    RFmotor.setDirection(DcMotor.Direction.REVERSE);
    RRmotor.setDirection(DcMotor.Direction.REVERSE);
    Wrist.setPosition(1);
    BucketDown = true;
    BucketDown = false;
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        ResetIMU();
        RaiseArm2();
        ExtendArm2();
        LiftRobot();
        Ramp2();
        scoop_in();
        Wrist_Up();
        Wrist_Middle();
        Wrist_Down();
        Y = gamepad1.left_stick_y;
        X = -gamepad1.left_stick_x;
        RX = gamepad1.right_stick_x * 1.1;
        botHeading = AngleUnit.DEGREES.fromUnit(AngleUnit.RADIANS, -imu.getAngularOrientation().firstAngle);
        cosHeading = Math.cos(botHeading / 180 * Math.PI);
        sinHeading = Math.sin(botHeading / 180 * Math.PI);
        rotX = X * cosHeading - Y * sinHeading;
        rotY = X * sinHeading + Y * cosHeading;
        telemetry.addData("Arm Extension (in)", ArmExtensionInches);
        telemetry.addData("Arm Limit", ArmLimit.isPressed());
        // AS5600 Encoder Angle In Degrees
        telemetry.addData("Arm Elevation", AS5600Block.AngleInDeg("AS5600sensor"));
        telemetry.addData("RightSitck Y", gamepad2.right_stick_y);
        telemetry.addData("Lift Power 1", LiftMotor1.getPower());
        telemetry.addData("Lift Power 2", LiftMotor2.getPower());
        telemetry.addData("RaiseArm Power", RaiseArm.getPower());
        telemetry.addData("Extend Arm Power", ExtendArm.getPower());
        telemetry.addData("LF Motor Power", LFmotor.getPower());
        telemetry.addData("RF Motor Power", RFmotor.getPower());
        telemetry.addData("LR Motor Power", LRmotor.getPower());
        telemetry.addData("RR Motor Power", RRmotor.getPower());
        telemetry.addData("BotHeading", botHeading);
        telemetry.addData("rotX", rotX);
        telemetry.addData("rotY", rotY);
        Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(RX))), 1));
        if (gamepad1.right_trigger > 0) {
          Multiplier = 0.9;
        } else if (gamepad1.left_trigger > 0) {
          Multiplier = 0.4;
        } else {
          Multiplier = 0.6;
        }
        LFmotor.setPower(Multiplier * -((rotY + rotX + RX) / Denominator));
        RFmotor.setPower(Multiplier * -(((rotY - rotX) - RX) / Denominator));
        LRmotor.setPower(Multiplier * -(((rotY - rotX) + RX) / Denominator));
        RRmotor.setPower(Multiplier * -(((rotY + rotX) - RX) / Denominator));
        telemetry.addData("Multiplier", Multiplier);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void RaiseArm2() {
    double ArmElevation;

    // AS5600 Encoder Angle In Degrees
    ArmElevation = AS5600Block.AngleInDeg("AS5600sensor");
    if (Math.abs(gamepad2.left_stick_y) > 0.1) {
      RaiseArm.setPower(gamepad2.left_stick_y * 0.8);
    } else {
      RaiseArm.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void ExtendArm2() {
    if (Math.abs(gamepad2.right_stick_y) > 0.1) {
      if (gamepad2.right_stick_y > 0 && !ArmLimit.isPressed()) {
        ExtendArm.setPower(0);
        ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      } else if (gamepad2.right_stick_y < 0 && ArmExtensionInches > 10) {
        ExtendArm.setPower(0);
      } else {
        ExtendArm.setPower(gamepad2.right_stick_y);
      }
    } else {
      ExtendArm.setPower(0);
    }
    ArmExtension = ExtendArm.getCurrentPosition();
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void initializeIMU() {
    BNO055IMU.Parameters imuParameters;

    // Create IMU parameters object
    imuParameters = new BNO055IMU.Parameters();
    // Setup IMU sensor mode
    imuParameters.mode = BNO055IMU.SensorMode.IMU;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    // Initialize IMU using parameters
    imu.initialize(imuParameters);
    // Report initialization to Driver Station
    telemetry.addData("Status", "IMU initialized, calibration started.");
    telemetry.update();
    // Wait for IMU to warm up
    sleep(1000);
    telemetry.addData("Status", "Calibration Complete");
    telemetry.addData("Action needed:", "Please press the start triangle");
    telemetry.update();
  }
}
