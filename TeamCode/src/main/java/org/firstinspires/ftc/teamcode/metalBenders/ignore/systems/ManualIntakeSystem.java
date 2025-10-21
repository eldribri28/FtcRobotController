package org.firstinspires.ftc.teamcode.metalBenders.ignore.systems;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.NONE;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.ignore.systems.types.SystemPriorityEnum;

public class ManualIntakeSystem extends AbstractSystem {

    @Override
    public SystemPriorityEnum getSystemPriority() {
        return SystemPriorityEnum.PRIORITY_3;
    }

    @Override
    protected void start() {
        //do nothing
    }

    @Override
    protected void process() {
        if(getHardwareManager().getGamepad().left_trigger > 0) {
            getHardwareManager().getIntakeMotor().setPower(1);
        } else if (getHardwareManager().getGamepad().left_bumper) {
            getHardwareManager().getIntakeMotor().setPower(-1);
        } else {
            getHardwareManager().getIntakeMotor().setPower(0);
        }
        ArtifactColorEnum intakeBallColor = getArtifactColor(getHardwareManager().getIntakeColorSensor(), "Intake Color Sensor");
        getDataManager().setIntakeBallColor(intakeBallColor);
        ArtifactColorEnum launcherBallColor = getArtifactColor(getHardwareManager().getLaunchColorSensor(), "Launcher Color Sensor");
        getDataManager().setLauncherBallColor(launcherBallColor);
    }

    @Override
    protected void shutdown() {
        //do nothing
    }

    private ArtifactColorEnum getArtifactColor(NormalizedColorSensor colorSensor, String sensorName) {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        ArtifactColorEnum artifactColorEnum;
        if (blue > green && red > 0.2 && blue > 0.3) {
            artifactColorEnum = PURPLE;
        } else if (green > red && green > blue && green > 0.3) {
            artifactColorEnum = GREEN;
        } else {
            artifactColorEnum = NONE;
        }
        addTelemetry(sensorName + " - RGB", "%6.3f %6.3f %6.3f", red, green, blue);
        addTelemetry(sensorName + " - Color", artifactColorEnum.name());
        return artifactColorEnum;
    }
}
