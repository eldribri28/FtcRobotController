package org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.ftc_2024;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.core.AS5600Block;

@TeleOp(name = "FieldCentricDriveMode (Blocks to Java)")
public class FieldCentricDriveMode extends LinearOpMode {

  private DcMotor ExtendArm;
  private DcMotor RaiseArm;
  private CRServo Grip1;
  private Servo Wrist;
  private DcMotor LiftMotor1;
  private DcMotor LiftMotor2;
  private DcMotor LFmotor;
  private DcMotor LRmotor;
  private DcMotor RFmotor;
  private DcMotor RRmotor;
  private IMU imu;
  private ColorSensor IntakeColor;
  private DistanceSensor IntakeColor_DistanceSensor;
  private TouchSensor ArmLimit;

  String colorSensorResult;
  String currentAutomation;
  String extensionMode;
  boolean flagAutoWristUp;
  boolean flagExtendRunToPosition;
  boolean flagArmLimit;
  int ArmMaxExtension;
  boolean flagAutoWristDown;
  ImuOrientationOnRobot orientationOnRobot;
  YawPitchRollAngles orientation;
  int RobotMaxLength;
  int ArmTicksPerInch;
  int Arm90DegOffset;
  int ArmBaseLength;
  double elevationMaxPower;
  double Multiplier;
  double extensionResetTime;
  double driveMaxPower;
  double extensionMaxPower;
  double botHeading;
  double rotX;
  double rotY;

  /**
   * Describe this function...
   */
  private void Arm_Extension() {
    double extensionLimitPower;

    if (ArmMaximumExtension() - ArmExtensionInches() <= 0 && currentAutomation.equals("None")) {
      if (ArmElevation() < 40) {
        ExtendArm.setPower(0.05);
      } else {
        ExtendArm.setPower(0.4);
      }
      extensionMode = "Automatic Retraction";
    } else {
      if (Math.abs(gamepad2.right_stick_y) >= 0.1) {
        currentAutomation = "None";
        if (gamepad2.right_stick_y < 0) {
          extensionMode = "Driver Extend";
          extensionLimitPower = extensionMaxPower * Range.clip((ArmMaximumExtension() - ArmExtensionInches()) / 4, 0.1, 1);
          ExtendArm.setPower(extensionLimitPower * gamepad2.right_stick_y);
        } else {
          extensionMode = "Driver Retract";
          ExtendArm.setPower(extensionMaxPower * gamepad2.right_stick_y);
        }
      } else {
        if (ArmElevation() < 20 && currentAutomation.equals("None")) {
          extensionMode = "Power Hold";
          ExtendArm.setPower(-0.005);
        } else {
          ExtendArm.setPower(0);
          extensionMode = "Hold";
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private double ArmMaximumExtension() {
    double extensionLimit;

    extensionLimit = Range.clip(RobotMaxLength - Math.cos(Math.abs(90 - ArmElevation()) / 180 * Math.PI) * ArmBaseLength, 0, ArmMaxExtension);
    return extensionLimit;
  }

  /**
   * Describe this function...
   */
  private void Arm_Elevation() {
    if (Math.abs(gamepad2.left_stick_y) >= 0.1) {
      if (ArmElevation() < 7 && gamepad2.left_stick_y < 0) {
        RaiseArm.setPower(0);
      } else if (ArmElevation() < 20) {
        RaiseArm.setPower(elevationMaxPower * gamepad2.left_stick_y * 0.6);
      } else {
        if (gamepad2.left_stick_y > 0 && ArmElevation() < 90) {
          RaiseArm.setPower(elevationMaxPower * 0.7 * gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y > 0 && ArmElevation() > 90) {
          RaiseArm.setPower(elevationMaxPower * 0.6 * gamepad2.left_stick_y);
        } else {
          RaiseArm.setPower(elevationMaxPower * gamepad2.left_stick_y);
        }
      }
      currentAutomation = "None";
    } else if (currentAutomation.equals("None")) {
      RaiseArm.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Automatic_Elevation(int elevationTarget) {
    int ArmTargetElevation;

    ArmTargetElevation = elevationTarget;
    if (ArmTargetElevation - ArmElevation() <= -0.4) {
      RaiseArm.setPower(-(Range.clip(Math.abs((ArmTargetElevation - ArmElevation()) / 10), 0.1, 1) * 0.5));
    } else if (ArmTargetElevation - ArmElevation() >= 0.4) {
      RaiseArm.setPower(Range.clip(Math.abs((ArmTargetElevation - ArmElevation()) / 10), 0.1, 1) * 0.5);
    } else {
      RaiseArm.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Intake_Stop() {
    Grip1.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void Intake_In() {
    if (CheckIntakeSensor().equals("None")) {
      Grip1.setDirection(CRServo.Direction.REVERSE);
      Grip1.setPower(1);
    } else {
      Grip1.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Intake_Out() {
    if (!CheckIntakeSensor().equals("None")) {
      Grip1.setDirection(CRServo.Direction.FORWARD);
      Grip1.setPower(0.6);
    } else {
      Grip1.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void Wrist_Up() {
    Wrist.setPosition(1);
  }

  /**
   * Describe this function...
   */
  private void Wrist_Down() {
    Wrist.setPosition(0.05);
  }

  /**
   * Describe this function...
   */
  private void Wrist_Middle() {
    Wrist.setPosition(0.4);
  }

  /**
   * Describe this function...
   */
  private double ArmExtensionInches() {
    double ArmExtensionInches2;

    ArmExtensionInches2 = -(ExtendArm.getCurrentPosition() / ArmTicksPerInch);
    return ArmExtensionInches2;
  }

  /**
   * Describe this function...
   */
  private void ScanButtons() {
    // Gamepad 1
    if (gamepad1.left_bumper) {
      LiftMotor1.setPower(-1);
      LiftMotor2.setPower(-1);
      ((DcMotorEx) LiftMotor1).setCurrentAlert(6, CurrentUnit.AMPS);
      ((DcMotorEx) LiftMotor2).setCurrentAlert(6, CurrentUnit.AMPS);
    } else if (gamepad1.right_bumper) {
      LiftMotor1.setPower(1);
      LiftMotor2.setPower(1);
      ((DcMotorEx) LiftMotor1).setCurrentAlert(6, CurrentUnit.AMPS);
      ((DcMotorEx) LiftMotor2).setCurrentAlert(6, CurrentUnit.AMPS);
    } else {
      LiftMotor1.setPower(0);
      LiftMotor2.setPower(0);
    }
    if (gamepad1.dpad_up) {
      ResetIMU();
    }
    if (gamepad1.right_trigger > 0) {
      Multiplier = driveMaxPower;
    } else if (gamepad1.left_trigger > 0) {
      Multiplier = driveMaxPower * 0.4;
    } else {
      Multiplier = driveMaxPower * 0.6;
    }
    // Gamepad 2
    if (gamepad2.a) {
      Wrist_Up();
    } else if (gamepad2.b) {
      Wrist_Down();
    } else if (gamepad2.x) {
      Wrist_Middle();
    }
    if (gamepad2.right_trigger > 0.5) {
      Intake_In();
    } else if (gamepad2.left_trigger > 0.5) {
      Intake_Out();
    } else {
      Intake_Stop();
    }
    if (gamepad2.dpad_up) {
      currentAutomation = "HighBasket";
      flagExtendRunToPosition = false;
    } else if (gamepad2.dpad_left) {
      currentAutomation = "Reset";
      flagExtendRunToPosition = false;
    } else if (gamepad2.dpad_down) {
      currentAutomation = "LowBasket";
      flagExtendRunToPosition = false;
    }
  }

  /**
   * Describe this function...
   */
  private void AutoWrist() {
    if (ArmElevation() < 20 && !flagAutoWristUp) {
      flagAutoWristUp = true;
      flagAutoWristDown = false;
      Wrist_Up();
    } else if (ArmElevation() > 20 && !flagAutoWristDown) {
      flagAutoWristUp = false;
      flagAutoWristDown = true;
      Wrist_Down();
    }
  }

  /**
   * Describe this function...
   */
  private void ArmAutomation() {
    if (currentAutomation.equals("HighBasket")) {
      Automatic_Elevation(7);
      Wrist_Up();
    } else if (currentAutomation.equals("LowBasket")) {
      Automatic_Elevation(7);
      Wrist_Up();
    } else if (currentAutomation.equals("Reset")) {
      Automatic_Elevation(90);
      Wrist_Down();
    } else {
      if (flagExtendRunToPosition) {
        flagExtendRunToPosition = false;
        ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void MotorPIDF() {
    ((DcMotorEx) LFmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
    ((DcMotorEx) LRmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
    ((DcMotorEx) RFmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
    ((DcMotorEx) RRmotor).setVelocityPIDFCoefficients(20, 0, 0, 10);
    ((DcMotorEx) ExtendArm).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(5, 0, 0, 10, MotorControlAlgorithm.PIDF));
  }

  /**
   * Describe this function...
   */
  private void ResetIMU() {
    imu.resetYaw();
  }

  /**
   * Describe this function...
   */
  private String CheckIntakeSensor() {
    NormalizedRGBA intakeSensorNormalizedColor;
    int intakeSensorColor;
    float intakeSensorHue;
    double intakeSensorDistance;

    intakeSensorNormalizedColor = ((NormalizedColorSensor) IntakeColor).getNormalizedColors();
    intakeSensorColor = intakeSensorNormalizedColor.toColor();
    intakeSensorHue = JavaUtil.rgbToHue(Color.red(intakeSensorColor), Color.green(intakeSensorColor), Color.blue(intakeSensorColor));
    intakeSensorDistance = Double.parseDouble(JavaUtil.formatNumber(IntakeColor_DistanceSensor.getDistance(DistanceUnit.INCH), 3));
    if (intakeSensorHue < 50 && intakeSensorDistance < 3) {
      colorSensorResult = "Red";
    } else if (intakeSensorHue < 150 && intakeSensorDistance < 3) {
      colorSensorResult = "Yellow";
    } else if (intakeSensorHue < 225 && intakeSensorDistance < 3) {
      colorSensorResult = "Blue";
    } else {
      colorSensorResult = "None";
    }
    return colorSensorResult;
  }

  /**
   * Describe this function...
   */
  private void Setup() {
    int colorSensorGain;
    int RobotCurrentLength;

    currentAutomation = "None";
    flagExtendRunToPosition = false;
    // Color Sensor Setup
    colorSensorGain = 5;
    colorSensorResult = "None";
    // If supported by the sensor, turn the light on in the beginning (it
    // might already be on anyway, we just make sure it is if we can).
    IntakeColor.enableLed(true);
    driveMaxPower = 0.9;
    Arm90DegOffset = 41;
    ArmTicksPerInch = 60;
    ArmBaseLength = 20;
    ArmMaxExtension = 29;
    RobotMaxLength = 36;
    RobotCurrentLength = 0;
    extensionResetTime = getRuntime();
    flagArmLimit = false;
    flagAutoWristUp = true;
    flagAutoWristDown = true;
    orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
    elevationMaxPower = 0.8;
    extensionMaxPower = 0.8;
    extensionMode = "Hold";
    ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RFmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RRmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    RaiseArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    LiftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LiftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RRmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ExtendArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RaiseArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ((DcMotorEx) ExtendArm).setTargetPositionTolerance(40);
//    Grip1.setPwmEnable();
//    Wrist.setPwmEnable();
  }

  /**
   * Describe this function...
   */
  private double ArmElevation() {
    double ArmElevation2;
    double correctedElevation;

    // AS5600 Encoder Angle In Degrees
    ArmElevation2 = AS5600Block.AngleInDeg("AS5600sensor");
    correctedElevation = ArmElevation2 - Arm90DegOffset;
    return correctedElevation;
  }

  /**
   * Describe this function...
   */
  private void ArmRetractLimit() {
    // Get the current time in milliseconds. The value returned represents
    // the number of milliseconds since midnight, January 1, 1970 UTC.
    if (!ArmLimit.isPressed() && System.currentTimeMillis() - extensionResetTime > 1000) {
      ExtendArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      if (flagExtendRunToPosition) {
        ExtendArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtendArm.setPower(0.1);
      } else {
        ExtendArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
      flagArmLimit = true;
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      extensionResetTime = System.currentTimeMillis();
      extensionMode = "Reset Limit";
    } else {
      flagArmLimit = false;
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    long matchstart;
    boolean flagOneShot;
    float Y;
    float X;
    double RX;
    double cosHeading;
    double sinHeading;
    double Denominator;

    ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
    RaiseArm = hardwareMap.get(DcMotor.class, "Raise Arm");
    Grip1 = hardwareMap.get(CRServo.class, "Grip1");
    Wrist = hardwareMap.get(Servo.class, "Wrist");
    LiftMotor1 = hardwareMap.get(DcMotor.class, "LiftMotor1");
    LiftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
    LFmotor = hardwareMap.get(DcMotor.class, "LFmotor");
    LRmotor = hardwareMap.get(DcMotor.class, "LRmotor");
    RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
    RRmotor = hardwareMap.get(DcMotor.class, "RRmotor");
    imu = hardwareMap.get(IMU.class, "imu");
    IntakeColor = hardwareMap.get(ColorSensor.class, "IntakeColor");
    IntakeColor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "IntakeColor");
    ArmLimit = hardwareMap.get(TouchSensor.class, "ArmLimit");

    Setup();
    initIMU();
    MotorPIDF();
    while (!isStarted()) {
      UpdateTelemetry();
    }
    waitForStart();
    resetRuntime();
    if (opModeIsActive()) {
      // Get the current time in milliseconds. The value returned represents
      // the number of milliseconds since midnight, January 1, 1970 UTC.
      matchstart = System.currentTimeMillis();
      flagOneShot = false;
      while (opModeIsActive()) {
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        if (System.currentTimeMillis() - matchstart > 3000 && !flagOneShot) {
          flagAutoWristDown = false;
          flagOneShot = true;
        }
        ArmRetractLimit();
        ScanButtons();
        Arm_Elevation();
        Arm_Extension();
        ArmAutomation();
        AutoWrist();
        Y = gamepad1.left_stick_y;
        X = -gamepad1.left_stick_x;
        RX = -(gamepad1.right_stick_x * 1.1);
        orientation = imu.getRobotYawPitchRollAngles();
        botHeading = -orientation.getYaw(AngleUnit.DEGREES);
        cosHeading = Math.cos(botHeading / 180 * Math.PI);
        sinHeading = Math.sin(botHeading / 180 * Math.PI);
        rotX = X * cosHeading - Y * sinHeading;
        rotY = X * sinHeading + Y * cosHeading;
        Denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(Y), Math.abs(X), Math.abs(RX))), 1));
        LFmotor.setPower(Multiplier * -((rotY + rotX + RX) / Denominator));
        RFmotor.setPower(Multiplier * -(((rotY - rotX) - RX) / Denominator));
        LRmotor.setPower(Multiplier * -(((rotY - rotX) + RX) / Denominator));
        RRmotor.setPower(Multiplier * -(((rotY + rotX) - RX) / Denominator));
        UpdateTelemetry();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void UpdateTelemetry() {
    telemetry.addData("Lift 1 Current (A)", ((DcMotorEx) LiftMotor1).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("Lift 2 Current (A)", ((DcMotorEx) LiftMotor2).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("Current Automation", currentAutomation);
    telemetry.addData("Arm Angle", ArmElevation());
    telemetry.addData("Arm Limit", !ArmLimit.isPressed());
    telemetry.addData("Arm Extension Mode", extensionMode);
    telemetry.addData("Arm Extension Target", ExtendArm.getTargetPosition());
    telemetry.addData("Arm Extension Power", ExtendArm.getPower());
    telemetry.addData("Arm Max Extension (in)", ArmMaximumExtension());
    telemetry.addData("Arm Current Extension (in)", ArmExtensionInches());
    telemetry.addData("RightSitck Y", gamepad2.right_stick_y);
    telemetry.addData("RF Motor Power", RFmotor.getPower());
    telemetry.addData("LF Motor Power", LFmotor.getPower());
    telemetry.addData("RR Motor Power", RFmotor.getPower());
    telemetry.addData("LR Motor Power", RRmotor.getPower());
    telemetry.addData("BotHeading", botHeading);
    telemetry.addData("rotX", rotX);
    telemetry.addData("rotY", rotY);
    telemetry.addData("Multiplier", Multiplier);
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void initIMU() {
    imu.initialize(new IMU.Parameters(orientationOnRobot));
    imu.resetYaw();
    orientation = imu.getRobotYawPitchRollAngles();
    sleep(1000);
  }
}
