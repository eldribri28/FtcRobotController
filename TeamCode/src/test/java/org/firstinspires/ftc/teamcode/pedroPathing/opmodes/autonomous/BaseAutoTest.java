package org.firstinspires.ftc.teamcode.pedroPathing.opmodes.autonomous;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.firstinspires.ftc.teamcode.pedroPathing.enums.ArtifactGroupEnum;
import org.junit.Test;

import java.io.File;
import java.util.List;


public class BaseAutoTest {

    private BaseAuto auto = new BlueNearAuto();
    private static final String TEST_CONFIG_FILE = "near.cfg";
    @Test
    public void testReadCfgFile() {
        File testConfigFile = new File(this.getClass().getClassLoader().getResource(TEST_CONFIG_FILE).getFile());
        List<ArtifactGroupEnum> groupsToCollect = auto.readCfgFile(testConfigFile);
        List<ArtifactGroupEnum> groupsToOpenClassifier = auto.artifactGroupsToEmptyClassifierAfterIntake;

        assertNotNull(groupsToCollect);
        assertEquals(5, groupsToCollect.size());
        assertEquals(ArtifactGroupEnum.PRELOAD, groupsToCollect.get(0));
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_1, groupsToCollect.get(1));
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_2, groupsToCollect.get(2));
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_3, groupsToCollect.get(3));
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_4, groupsToCollect.get(4));

        assertNotNull(groupsToOpenClassifier);
        assertEquals(2, groupsToOpenClassifier.size());
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_2, groupsToOpenClassifier.get(0));
        assertEquals(ArtifactGroupEnum.ARTIFACT_GROUP_4, groupsToOpenClassifier.get(1));
    }
}
