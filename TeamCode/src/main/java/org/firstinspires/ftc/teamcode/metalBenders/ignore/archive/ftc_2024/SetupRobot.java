package org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.ftc_2024;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.core.AS5600Block;

@TeleOp(name = "SetupRobot (Blocks to Java)")
public class SetupRobot extends LinearOpMode {

  private TouchSensor ArmLimit;
  private DcMotor ExtendArm;
  private DcMotor RaiseArm;
  private Servo Wrist;

  int elevation90DegOffset;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    int elevationTarget;

    ArmLimit = hardwareMap.get(TouchSensor.class, "ArmLimit");
    ExtendArm = hardwareMap.get(DcMotor.class, "ExtendArm");
    RaiseArm = hardwareMap.get(DcMotor.class, "Raise Arm");
    Wrist = hardwareMap.get(Servo.class, "Wrist");

    // Put initialization blocks here.
    elevation90DegOffset = 124;
    elevationTarget = 112;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (ArmLimit.isPressed() && !isStopRequested()) {
        ExtendArm.setPower(0.6);
        UpdateTelemetry();
      }
      Wrist_Down();
      sleep(1000);
      while (Math.abs(elevationTarget - ArmElevation()) >= 0.4 && !isStopRequested()) {
        if (elevationTarget - ArmElevation() < 0) {
          RaiseArm.setPower(-0.3);
        } else if (elevationTarget - ArmElevation() > 0) {
          RaiseArm.setPower(0.3);
        }
        UpdateTelemetry();
      }
      requestOpModeStop();
    }
  }

  /**
   * Describe this function...
   */
  private void UpdateTelemetry() {
    telemetry.addData("Arm Limit Pressed", !ArmLimit.isPressed());
    telemetry.addData("Arm Elevation (deg)", ArmElevation());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void Wrist_Down() {
    String WristState;

    Wrist.setPosition(0.05);
    WristState = "Down";
  }

  /**
   * Describe this function...
   */
  private double ArmElevation() {
    double ArmElevation2;
    double correctedElevation;

    // AS5600 Encoder Angle In Degrees
    ArmElevation2 = AS5600Block.AngleInDeg("AS5600sensor");
    correctedElevation = ArmElevation2 - elevation90DegOffset;
    return correctedElevation;
  }
}
