package org.firstinspires.ftc.teamcode.metalBenders.ignore.archive.ftc_2025;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Disabled
@TeleOp(name = "OperatorDriveMode")
public class OperatorDriveMode extends LinearOpMode {

  private HardwareManagementSystem hardwareManagementSystem;
  private List<Thread> systemThreads;
  private boolean isSystemThreadsStarted = false;
  private final Map<String, String> telemetryData = new ConcurrentHashMap<>();

  @Override
  public void runOpMode() {
    initialize();
    waitForStart();
    resetRuntime();
    while (opModeIsActive()) {
      if(!isSystemThreadsStarted) {
        startSystem();
      }
      updateRuntime();
      updateTelemetry();
    }
    stopSystem();
  }

  private List<Thread> createSystemThreads() {
    return List.of(
            new Thread(new ArmControlSystem(hardwareManagementSystem, telemetryData)),
            new Thread(new DriveControlSystem(hardwareManagementSystem, telemetryData)));
  }

  private void updateRuntime() {
    double totalSeconds = getRuntime();
    telemetryData.put(
            "Runtime",
            String.format(
                    Locale.ENGLISH,
                    "%02.0f:%02.0f",
                    totalSeconds / 60,
                    totalSeconds % 60));
  }

  private void updateTelemetry() {
    List<String> keys = new ArrayList<>(telemetryData.keySet());
    Collections.sort(keys);
    keys.forEach(key -> telemetry.addData(key, telemetryData.get(key)));
    telemetry.update();
  }

  private void startSystem() {
    systemThreads.forEach(Thread::start);
    isSystemThreadsStarted = true;
  }

  private void stopSystem() {
    systemThreads.forEach(Thread::interrupt);
    isSystemThreadsStarted = false;
  }

  private void initialize() {
    hardwareManagementSystem = new HardwareManagementSystem(hardwareMap, gamepad1);
    systemThreads = createSystemThreads();
  }
}
