package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.BOT_HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.CAMERA_FIELD_H;

public class AutonTelemetry {

    

    public static void refreshTelemetry() {
        Telemetry telemetry = null;
        telemetry.addData("Robot Heading Offset (deg)", BOT_HEADING_OFFSET);
        telemetry.addLine(String.format("Robot Position (x,y,h) (m,m,deg) %6.3f %6.3f %6.3f", CAMERA_FIELD_X, CAMERA_FIELD_Y, CAMERA_FIELD_H));
        telemetry.update();
    }



}
