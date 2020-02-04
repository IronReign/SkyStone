package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="open cv skystone test")
public class SkystoneOpenCVTest extends LinearOpMode {

    SkystoneOpenCVIntegration vp;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "configuration");
        telemetry.update();

        while (!isStarted()) {
            vp = new SkystoneOpenCVIntegration();
            vp.initializeVision(hardwareMap, telemetry, false, Viewpoint.WEBCAM, true);

            telemetry.addData("Status", "initialized vision");
            telemetry.update();
        }

        telemetry.addData("Status", "Started");
        telemetry.update();

        SkystoneTargetInfo gp = null;

        while (opModeIsActive()) {
            SkystoneTargetInfo target = vp.detect();
            if (target.finished)
                gp = target;
            telemetry.addData("VisionDetection", "%s", gp);
            telemetry.addData("Found", "%s", target.finished ? "YES" : "NO");
            telemetry.update();
        }
        vp.shutdownVision();
    }
}