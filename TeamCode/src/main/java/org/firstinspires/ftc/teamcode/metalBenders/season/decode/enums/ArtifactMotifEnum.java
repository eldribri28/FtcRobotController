package org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums;

import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.GREEN;
import static org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum.PURPLE;

public enum ArtifactMotifEnum {
    GREEN_PURPLE_PURPLE(GREEN, PURPLE, PURPLE),
    PURPLE_GREEN_PURPLE(PURPLE, GREEN, PURPLE),
    PURPLE_PURPLE_GREEN(PURPLE, PURPLE, GREEN),
    UNKNOWN(null, null, null);

    private final ArtifactColorEnum position1ArtifactColor;
    private final ArtifactColorEnum position2ArtifactColor;
    private final ArtifactColorEnum position3ArtifactColor;

    ArtifactMotifEnum(
            ArtifactColorEnum position1ArtifactColor,
            ArtifactColorEnum position2ArtifactColor,
            ArtifactColorEnum position3ArtifactColor) {
        this.position1ArtifactColor = position1ArtifactColor;
        this.position2ArtifactColor = position2ArtifactColor;
        this.position3ArtifactColor = position3ArtifactColor;
    }

    public ArtifactColorEnum getPosition1ArtifactColor() {
        return position1ArtifactColor;
    }

    public ArtifactColorEnum getPosition2ArtifactColor() {
        return position2ArtifactColor;
    }

    public ArtifactColorEnum getPosition3ArtifactColor() {
        return position3ArtifactColor;
    }
}
