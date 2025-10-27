package org.firstinspires.ftc.teamcode.metalBenders.season.decode.util;

import static com.sun.tools.javac.main.Option.H;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.BOT_HEADING_OFFSET;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_TIME;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROTATE_PID_LAST_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_ACCUMULATED_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_TIME;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.DRIVE_PID_LAST_ERROR;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_X;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_Y;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.properties.GlobalVars.ROBOT_FIELD_H;

public class AutonTelemetry {

    

    public static void refreshTelemetry() {
        Telemetry telemetry = null;
        telemetry.addData("Robot Heading Offset (deg)", BOT_HEADING_OFFSET);
        telemetry.addLine(String.format("Robot Position (x,y,h) (m,m,deg) %6.3f %6.3f %6.3f", ROBOT_FIELD_X, ROBOT_FIELD_Y, ROBOT_FIELD_H));
        telemetry.update();
    }



}
