package org.firstinspires.ftc.teamcode.metalBenders.ignore.data;

import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactColorEnum;
import org.firstinspires.ftc.teamcode.metalBenders.season.decode.enums.ArtifactMotifEnum;

import java.util.concurrent.locks.ReentrantReadWriteLock;

public class DataManager {

    private ArtifactMotifEnum artifactMotif = ArtifactMotifEnum.UNKNOWN;
    private final ReentrantReadWriteLock artifactMotifLock = new ReentrantReadWriteLock();
    private ArtifactColorEnum intakeBallColor = ArtifactColorEnum.NONE;
    private final ReentrantReadWriteLock intakeColorLock = new ReentrantReadWriteLock();
    private ArtifactColorEnum launcherBallColor = ArtifactColorEnum.NONE;
    private final ReentrantReadWriteLock launcherColorLock = new ReentrantReadWriteLock();

    public ArtifactMotifEnum getArtifactMotif() {
        artifactMotifLock.readLock().lock();
        try {
            return artifactMotif;
        } finally {
            artifactMotifLock.readLock().unlock();
        }
    }

    public void setArtifactMotif(ArtifactMotifEnum artifactMotifEnum) {
        artifactMotifLock.writeLock().lock();
        try {
            this.artifactMotif = artifactMotifEnum;
        } finally {
            artifactMotifLock.writeLock().unlock();
        }
    }

    public ArtifactColorEnum getIntakeBallColor() {
        intakeColorLock.readLock().lock();
        try {
            return intakeBallColor;
        } finally {
            intakeColorLock.readLock().unlock();
        }
    }

    public void setIntakeBallColor(ArtifactColorEnum intakeBallColor) {
        intakeColorLock.writeLock().lock();
        try {
            this.intakeBallColor = intakeBallColor;
        } finally {
            intakeColorLock.writeLock().unlock();
        }
    }

    public ArtifactColorEnum getLauncherBallColor() {
        launcherColorLock.readLock().lock();
        try {
            return launcherBallColor;
        } finally {
            launcherColorLock.readLock().unlock();
        }
    }

    public void setLauncherBallColor(ArtifactColorEnum launcherBallColor) {
        launcherColorLock.writeLock().lock();
        try {
            this.launcherBallColor = launcherBallColor;
        } finally {
            launcherColorLock.writeLock().unlock();
        }
    }
}
