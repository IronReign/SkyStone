package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.samples.ConceptVuforiaSkyStoneNavigationWebcam;

public class VisionProvidersSkystone {

    private int posState;


    public VisionProvidersSkystone() {


    public void reset() {
    }

    public SkystonePos detect() {
        if (vs.getY() > -14.0f || vs.getY() <= -7.0f) {
            return SkystonePos.LEFT;
        }
        if (vs.getY() > -14.0f || vs.getY() <= -7.0f) {
            return SkystonePos.MIDDLE;
        }
        if (vs.getY() > -14.0f || vs.getY() <= -7.0f) {
            return SkystonePos.RIGHT;
        } else
            return SkystonePos.NONE_FOUND;
    }


    public void shutdownVision() {
    }
}
