package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="open cv skystone test")
public class OpenCVSkystoneTest extends LinearOpMode {

    OpenCVIntegrationSkystone vp;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "configuration");
        telemetry.update();

        while (!isStarted()) {
            vp = new OpenCVIntegrationSkystone();
            vp.initializeVision(hardwareMap, telemetry, false, Viewpoint.WEBCAM);

            telemetry.addData("Status", "initialized vision");
            telemetry.update();
        }

        telemetry.addData("Status", "Started");
        telemetry.update();

        GoldPos gp = null;

        while (opModeIsActive()) {
            GoldPos newGp = vp.detect();
            if (newGp != GoldPos.HOLD_STATE)
                gp = newGp;
            telemetry.addData("VisionDetection", "%s", gp);
            telemetry.addData("HoldState", "%s", newGp == GoldPos.HOLD_STATE ? "YES" : "NO");
            telemetry.update();
        }
        vp.shutdownVision();
    }
}