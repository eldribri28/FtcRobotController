package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auton Menu", group="Menu")
public class AutonMenu extends LinearOpMode {

    private List<String> autonList = new ArrayList<>();
    private BufferedWriter writer;

    // Predefined list of options
    private final String[] NearFarOptions = {
            "Near",
            "Far"
    };
    private final String[] menuOptions = {
            "PRELOAD",
            "ARTIFACT_GROUP_1",
            "ARTIFACT_GROUP_1+OPEN_CLASSIFIER",
            "ARTIFACT_GROUP_2",
            "ARTIFACT_GROUP_2+OPEN_CLASSIFIER",
            "ARTIFACT_GROUP_3",
            "ARTIFACT_GROUP_3+OPEN_CLASSIFIER",
            "ARTIFACT_GROUP_4",
            "ARTIFACT_GROUP_4+OPEN_CLASSIFIER",
            "ARTIFACT_GROUP_4_DIRECT",
            "ARTIFACT_GROUP_4_DIRECT+OPEN_CLASSIFIER"
    };

    private int selectedIndex = 0;

    // Gamepad state trackers for debouncing
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean lastGamepadA = false;
    private boolean lastGamepadB = false;
    private boolean lastGamepadX = false;
    private int menuLevel = 0;
    private boolean selectionFinished = false;
    private String autonNearFar = "";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized. Use Gamepad D-Pad.");
        telemetry.update();

        // Wait for the driver to hit INIT on the Driver Station
        waitForStart();

        while (opModeIsActive() && !selectionFinished) {

            readGamepad();
            displayMenu();

        }

    }

    private void readGamepad() {
        // Get current gamepad state
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;
        boolean currentA = gamepad1.a;
        boolean currentB = gamepad1.b;
        boolean currentX = gamepad1.x;

        // Handle D-pad Up (Previous Option)
        if (currentDpadUp && !previousDpadUp) {
            selectedIndex--;
        }

        // Handle D-pad Down (Next Option)
        if (currentDpadDown && !previousDpadDown) {
            selectedIndex++;
        }

        if (menuLevel == 0) {
            // Handle A button (Select)
            if (currentA && !lastGamepadA) {
                List<String> autonList = new ArrayList<>();
                autonNearFar = NearFarOptions[selectedIndex];
                menuLevel = 1;
            }
            if (selectedIndex >= NearFarOptions.length) {
                selectedIndex = 0; // Wrap around to top
            }
            if (selectedIndex < 0) {
                selectedIndex = NearFarOptions.length - 1; // Wrap around to bottom
            }
        } else if (menuLevel == 1) {
            if (currentA && !lastGamepadA) {
                autonList.add(menuOptions[selectedIndex]);
            }
            if (selectedIndex >= menuOptions.length) {
                selectedIndex = 0; // Wrap around to top
            }
            if (selectedIndex < 0) {
                selectedIndex = menuOptions.length - 1; // Wrap around to bottom
            }
            if (currentB && !lastGamepadB) {
                List<String> autonList = new ArrayList<>();
                menuLevel = 0;
            }
            if (currentX && !lastGamepadX) {
                writeCfgFile(autonNearFar, autonList);
                selectionFinished = true;
            }

        }

        // Update previous button states for the next loop
        previousDpadUp = currentDpadUp;
        previousDpadDown = currentDpadDown;
        lastGamepadA = currentA;
        lastGamepadB = currentB;
        lastGamepadX = currentX;
    }

    private void displayMenu() {
        telemetry.addLine("--- AUTON MENU ---");
        if (menuLevel == 0) {
            telemetry.addLine("Select Near / Far");
            for (int i = 0; i < NearFarOptions.length; i++) {
                if (i == selectedIndex) {
                    // Highlight the currently selected option with arrows
                    telemetry.addData("->", ">> " + NearFarOptions[i] + " <<");
                } else {
                    telemetry.addData("  ", "   " + NearFarOptions[i]);
                }
            }
        }
        if (menuLevel == 1) {
            String autonMenuString = "Creating " + autonNearFar + "Auton";
            telemetry.addLine(autonMenuString);
            for (int i = 0; i < menuOptions.length; i++) {
                if (i == selectedIndex) {
                    // Highlight the currently selected option with arrows
                    telemetry.addData("->", ">> " + menuOptions[i] + " <<");
                } else {
                    telemetry.addData("  ", "   " + menuOptions[i]);
                }
            }
            String selectedAutons = String.join(", ", autonList);
            telemetry.addData("Auton List", selectedAutons);
        }
        telemetry.addLine("");
        telemetry.addLine("--------------------");
        telemetry.addLine("A: Select, B: Back, X: Save");
        telemetry.update();
    }

    private void writeCfgFile(String autonType, List autonList) {
        File sdcard = new File("/sdcard/FIRST/config");
        File cfgFile = new File(sdcard, "blank.cfg");
        if (autonType == "Near") {
            cfgFile = new File(sdcard, "near.cfg");
        } else {
            cfgFile = new File(sdcard, "far.cfg");
        }
        try {
            writer = new BufferedWriter(new FileWriter(cfgFile, false));
            for (int i = 0; i < autonList.size(); i++) {
                writer.write(autonList.get(i).toString());
                if (i < autonList.size() - 1) {
                    writer.newLine(); // Automatically handles system-specific line breaks
                }
            }
            writer.flush();
            writer.close();
        } catch (IOException e) {
            telemetry.addData("Error", e);
        }
    }
}


