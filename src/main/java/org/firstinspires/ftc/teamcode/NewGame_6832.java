/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.Locale;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.HINT;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the tertiaryAuto or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="GnuGame_6832", group="Linear Opmode")  // @Autonomous(...) is the other common choice
//  @Autonomous

public class NewGame_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Pose robot = new Pose();

    SoundPlayer deadShotSays = SoundPlayer.getInstance();




    private boolean active = true;
    boolean joystickDriveStarted = false;

    private int autoState = 0;
    private int beaconState = 0;

    private int flingNumber = 0;
    private boolean shouldLaunch = false;
    private boolean isBlue = false;
    private boolean capMode = false;
    private double pwrDamper = 1.0;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;

    Orientation angles;

    private int state = 0;
    private boolean runAutonomous = true;
    private long autoTimer = 0;
    private long launchTimer = 0;
    private long autoDelay = 0;
    private boolean[] buttonSavedStates = new boolean[11];
    //private boolean[] buttonCurrentState = new boolean[8];
    private boolean slowMode = false;

    private boolean runDemo = false;
    private boolean runBeaconTestLeft = true;


    private int pressedPosition = 750; //Note: find servo position value for pressing position on servoGate
    private int relaxedPosition = 2250; //Note: find servo position value for relaxing position on servoGate

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    double testableDouble = robot.KpDrive;
    double testableHeading = 0;
    boolean testableDirection = true;

    private int a = 0; //collect (particle mode), manual cap lower (cap mode)
    private int b = 1; //eject (particle mode)
    private int x = 2; //launch (particle mode)
    private int y = 3; //spin up (particle mode), manual cap raise (cap mode)
    private int dpad_down = 4; //no function (particle mode), lowers cap to next lowest preset (cap mode)
    private int dpad_up = 5; //no function (particle mode), raises cap to next highest preset (cap mode)
    private int dpad_left = 6; //toggles between particle and cap mode (both modes)
    private int dpad_right = 7; //no function
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this.hardwareMap, isBlue);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        configureDashboard();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        Vuforia.setHint (HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

//        VuforiaTrackables beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
//        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
//        VuforiaTrackableDefaultListener legos = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();

        VuforiaTrackables beaconTargets = locale.loadTrackablesFromAsset("FTC_2016-17");
        beaconTargets.get(0).setName("Wheels");
        beaconTargets.get(1).setName("Tools");
        beaconTargets.get(2).setName("Lego");
        beaconTargets.get(3).setName("Gears");


        VuforiaTrackable redNearTarget = beaconTargets.get(3);
        redNearTarget.setName("redNear");  // Gears

        VuforiaTrackable blueNearTarget  = beaconTargets.get(0);
        blueNearTarget.setName("blueNear");  // Wheels

        VuforiaTrackable redFarTarget = beaconTargets.get(1);
        redFarTarget.setName("redFar");  // Tools

        VuforiaTrackable blueFarTarget  = beaconTargets.get(2);
        blueFarTarget.setName("blueFar");  // Legos

//        waitForStart(); //this is commented out but left here to document that we are still doing the functions that waitForStart() normally does, but needed to customize it.

        while(!isStarted()){    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            //beacons.activate();
            beaconTargets.activate();

            stateSwitch();

            if(gamepad1.a){
                flingNumber = 1;
            }
            if(gamepad1.b){
                flingNumber = 2;
            }
            if(gamepad1.y){
                flingNumber = 3;
            }
            if(toggleAllowed(gamepad1.x,x)) {

                    isBlue = !isBlue;

            }
            if(toggleAllowed(gamepad1.dpad_down,dpad_down)){

                autoDelay--;
                if(autoDelay < 0) autoDelay = 15;

            }
            if(toggleAllowed(gamepad1.dpad_up, dpad_up)){

                autoDelay++;
                if(autoDelay>15) autoDelay = 0;

            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("Status", "Number of throws: " + Integer.toString(flingNumber));
            telemetry.addData("Status", "Side: " + getAlliance());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }



        runtime.reset();

        if(!runAutonomous){
            state = 1;
        }



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();
            stateSwitch();
            if(active) {
                switch(state){
                    case 0: //main tertiaryAuto function that scores 1 or 2 balls and toggles both beacons
                        joystickDriveStarted = false;
                        autonomous(1);
                        break;
                    case 1: //this is the tertiaryAuto we use if our teamates can also go for the beacons more reliably than we can; scores 2 balls and pushes the cap ball, also parks on the center element
                        joystickDriveStarted = false;
                        secondaryAuto();
                        break;
                    case 2: //code for tele-op control
                        joystickDrive();
                        break;
                    case 3:
                        break;
                    case 4:
                        vuTest((VuforiaTrackableDefaultListener)redNearTarget.getListener(),500);
                        break;
                    case 5: //provides data for forwards/backwards calibration
                        joystickDriveStarted = false;
                        if(robot.driveForward(false, 1, .5)) active = false;
                        break;
                    case 6: //provides data for left/right calibration
                        joystickDriveStarted = false;
                        if(robot.getAverageAbsTicks() < 2000){
                            robot.driveMixer(0,1,0);
                        }
                        else robot.driveMixer(0,0,0);
                        break;
                    case 7: //demo mode
                        demo();
                        break;
                    case 8: //tertiaryAuto demo mode
                        //tertiaryAuto(1.0);
                        //break;
                        testableHeading = robot.getHeading();
                        state++;
                    case 9:
                        testableAuto(testableHeading);
                        break;
                }
                robot.updateSensors();
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void vuTest(VuforiaTrackableDefaultListener  beacontarget, double distance){
        robot.driveToBeacon(beacontarget, 500, 0.8, false);
    }
    public void demo(){
        robot.MaintainHeading(gamepad1.x);

    }

//    public void vuTest(){
//        int config = VisionUtils.NOT_VISIBLE;
//        try{
//            config = VisionUtils.waitForBeaconConfig(
//                    getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565),
//                    wheels, locale.getCameraCalibration(), 5000);
//            telemetry.addData("Beacon", config);
//            Log.i(TAG, "runOp: " + config);
//        } catch (Exception e){
//            telemetry.addData("Beacon", "could not not be found");
//        }//catch
//
//        if (config == VisionUtils.BEACON_BLUE_RED) {
//            RC.t.speakString("Left");
//        } else {
//            RC.t.speakString("Right");
//        }//else
//
//    }

    public void joystickDrive(){

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */
        if(toggleAllowed(gamepad1.dpad_left, dpad_left)){
            capMode = !capMode;
        }
        if (capMode) pwrDamper = -0.25;
        else pwrDamper = 1.0;

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            joystickDriveStarted = true;
        }
        pwrFwd = pwrDamper * -gamepad1.left_stick_y;
        pwrStf = pwrDamper * -gamepad1.left_stick_x;
        pwrRot = pwrDamper * -gamepad1.right_stick_x;

        if (!runDemo)
            robot.driveMixer(pwrFwd, pwrStf, pwrRot);

        if(!capMode) {

            //toggle the particle conveyor on and off - quick and dirty
            if (toggleAllowed(gamepad1.a, a)) {
                robot.particle.collectToggle();
            }

            if (toggleAllowed(gamepad1.b, b)) {
                robot.particle.eject();
            }
            if (toggleAllowed(gamepad1.x, x)) {
                robot.particle.launchToggle();
            }
            if (toggleAllowed(gamepad1.y, y)) {
                robot.particle.spinUpToggle();
            }
        }
        else{
            if (toggleAllowed(gamepad1.dpad_up, dpad_up)) {
                robot.cap.cycleUp();
            }
            if (toggleAllowed(gamepad1.dpad_down, dpad_down)) {
                robot.cap.cycleDown();
            }
            if (gamepad1.y){
                robot.cap.raise(.75);
            }
            else if (gamepad1.a){
                robot.cap.lower(.75);
            }
            else{
                robot.cap.stop();
            }
        }
//        if(capMode){
//            if (!runDemo)
//                robot.driveMixer(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x
//        }

//
//        if(toggleAllowed(gamepad1.y,3)){
//
//            if (robot.particle.isStopped()) {
//                robot.particle.restart();
//            } else {
//                robot.particle.emergencyStop();
//            }
//        }

//        if(toggleAllowed(gamepad1.dpad_down,4)){
//
//            robot.particle.halfCycle();
//
//        }

//        if(toggleAllowed(gamepad1.dpad_up, 5)){
//            runDemo = !runDemo;
//            robot.resetBeaconPresserState();
//        }
//        if(toggleAllowed(gamepad1.dpad_left, 6) && !runDemo){
//            isBlue = !isBlue;
//        }
//        if(toggleAllowed(gamepad1.dpad_right, 7) && !runDemo){
//            runBeaconTestLeft = !runBeaconTestLeft;
//        }
//
//        if(runDemo) {
//            if(robot.pressAllianceBeacon(isBlue, runBeaconTestLeft))
//                runDemo = false;
//            telemetry.addData("Status", "Test beacon on left Side: " + runBeaconTestLeft);
//            telemetry.addData("Status", "Side: " + getAlliance());
//        }
//        if(toggleAllowed(gamepad1.dpad_up, 5)) {
//            robot.setHeading(90);
//        }
//
//        if(toggleAllowed(gamepad1.dpad_left, 6)) {
//            runDemo = !runDemo;
//        }
//
//        if(runDemo){
//            robot.RotateIMU(.01, 0, 0, Integer.MAX_VALUE, 0);
//        }


//        if(gamepad1.dpad_up)
//            robot.cap.raise(1);
//
//        else if(gamepad1.dpad_down)
//            robot.cap.lower(1);
//
//        else
//            robot.cap.stop();

//        if(shouldLaunch){
//            robot.particle.launchToggle();
//        }
        robot.particle.updateCollection();
    }

    public void resetAuto(){
        autoState = 0;
        autoTimer = 0;
        robot.ResetTPM();
    }

    public void autonomous(double scaleFactor){ //this auto starts pointed towards the space between the beacons
        if (autoState == 0 && autoTimer == 0) autoTimer = futureTime(30f);
        if ( autoTimer > System.nanoTime()) {
            switch (autoState) {
                case -1: //sit and spin - do nothing
                    break;

                case 0: //reset all the motors before starting autonomous


                    robot.setTPM_Forward((long) (robot.getTPM_Forward() * scaleFactor));
                    robot.setTPM_Strafe((long) (robot.getTPM_Strafe() * scaleFactor));
                    robot.resetMotors(true);

                    autoState++;


                    //launchTimer = futureTime(0.75f);
                    deadShotSays.play(hardwareMap.appContext, R.raw.a0);
                    if(!isBlue) robot.setHeading(180);
                    else robot.setHeading(270);
                    robot.particle.spinUp();
                    robot.particle.launchEnd(); //gate should remain closed

                    break;
                case 1://drive forward for proper shot distance to vortex
                    if(robot.driveForward(false, .2, .5)) { //drive away from wall for a clear turn (formerly .35m, now .65m)
                        robot.resetMotors(true);
                        deadShotSays.play(hardwareMap.appContext, R.raw.a01);
                        launchTimer = futureTime(3.5f);
                        robot.particle.launchBegin();
                        autoState++;
                    }
                    break;

                case 2:
                    if(System.nanoTime() >launchTimer){ //particles should have launched, shut down launching
                        robot.particle.launchEnd();
                        robot.particle.spinDown();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a02);
                        autoState++;
                    }
                    break;

                case 3:
                    if(robot.driveForward(false, .0, .5)) { //back up a bit for a better turn to wall
                        robot.resetMotors(true);
                        deadShotSays.play(hardwareMap.appContext, R.raw.a03);
                        autoState++;
                    }
                    break;

                case 4:
                    if(robot.RotateIMU(45, 2.5)) { //should now be pointing at the target beacon wall
                        robot.resetMotors(true);
                        robot.particle.collectStart();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a04);
                        autoState++;
                    }
                    break;
                case 5:
                    if(robot.driveForward(!isBlue, 2, .75)) { //drive to middle of beacon wall
                        //robot.resetMotors(true);
                        robot.particle.collectStop();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a05);
                        autoState++;
                    }
                    break;
                case 6:
                    if(robot.driveForward(!isBlue, 2.80, .35)) { //drive to middle of beacon wall
                        robot.resetMotors(true);
                        robot.particle.collectStop();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a06);
                        autoState++;
                    }
                    break;
                case 7:

                    if(isBlue){ //align with blue beacon wall
                        if(robot.RotateIMU(90, 1.5)) {
                            robot.resetMotors(true);
                            deadShotSays.play(hardwareMap.appContext, R.raw.a07);
                            autoState++;
                        }
                    }
                    else{ //align with red beacon wall
                        if(robot.RotateIMU(0, 1.5)) {
                            robot.resetMotors(true);
                            deadShotSays.play(hardwareMap.appContext, R.raw.a07);
                            autoState++;
                        }
                    }
                    break;
                case 8: //press the first beacon
                    if (robot.pressAllianceBeacon(isBlue, false)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 9: //drive up next to the second beacon
                    if(isBlue){
                        if(robot.DriveIMUDistance(.01, .5, 90, false, .85)){
                            robot.resetMotors(true);
                            autoState++;
                        }
                    }
                    else{
                        if(robot.DriveIMUDistance(.01, .5, 0, true, .85)){
                            robot.resetMotors(true);
                            autoState++;
                        }
                    }
                    break;
                case 10: //press the second beacon
                    if (robot.pressAllianceBeacon(isBlue, true)) {
                        robot.resetMotors(true);
                        autoState++;
                    }
                    break;
                case 11: //turn back to 45 to point to vortex
                    if(robot.RotateIMU(45, 2.5)) { //should now be pointing at the target beacon wall
                        robot.resetMotors(true);
                        robot.particle.collectStart();
                        deadShotSays.play(hardwareMap.appContext, R.raw.a11);
                        autoState++;
                    }
                    break;
                case 12: //drive back to push cap ball and park
                    if(robot.driveForward(isBlue, 1.5, .75)) { //to center vortex

                        robot.particle.collectStop();
                        robot.resetMotors(true);
                        deadShotSays.play(hardwareMap.appContext, R.raw.a12);
                        autoState++;
                    }
                    break;
//                case 13:
//                    if(isBlue){
//                        if(robot.RotateIMU(180, .25)) autoState++;
//                    }
//                    else{
//                        if(robot.RotateIMU(90, .25)) autoState++;
//                    }
//                    break;
//                case 14:
//                    if(robot.driveForward(true, .25, .5)){
//                        robot.resetMotors(true);
//                        for (int n = 0; n < flingNumber; n++)
//                            robot.particle.fling();
//                        autoState++;
//                    }
//                    break;
                default:
                    robot.ResetTPM();
                    break;
            }
            robot.particle.updateCollection();
        }
        else{
            robot.particle.emergencyStop();
            robot.driveMixer(0, 0, 0);
            //switch to teleop
            autoTimer = 0;
            state = 2;
            active = true;
        }
    }


    public void stateSwitch(){

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if(toggleAllowed(gamepad1.left_bumper,left_bumper)) {

            state--;
            if (state < 0) {
                state = 9 ;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();

        }

        if (toggleAllowed(gamepad1.right_bumper,right_bumper)) {

            state++;
            if (state > 9) {
                state = 0;
            }
            robot.resetMotors(true);
            active = false;
            resetAuto();

        }

        if(toggleAllowed(gamepad1.start, startBtn)) {
            robot.resetMotors(true);
            active = !active;

        }
    }

    public void secondaryAuto(){
        switch(autoState){
            case 0:
                autoDelay = futureTime(autoDelay);
                robot.resetMotors(true);
                autoState++;
                break;
            case 1:
                if(System.nanoTime() > autoDelay) {
                    robot.setHeading(135);
                    robot.resetMotors(true);
                    robot.particle.spinUp();
                    autoState++;
                }
                break;
            case 2:
                if(robot.driveForward(false, .85, 1)){
                    robot.resetMotors(true);
                    launchTimer = futureTime((float)2.5);
                    robot.particle.launchBegin();
                    autoState++;
                }
                break;
            case 3:
                if(System.nanoTime() > launchTimer){
                    robot.particle.launchEnd();
                    robot.particle.spinDown();
                    autoState++;
                }
                //robot.motorConveyor.setPower(90);
                break;
            case 4:
                if(robot.driveForward(false, .8, 1)){
                    robot.resetMotors(true);
                    robot.motorConveyor.setPower(0);
                    autoState++;
                break;
                }
            default:
                break;
        }
    }

    public void calibrateLift(){
        if(gamepad1.dpad_up){
            robot.cap.raise(.75);
        }
        else if(gamepad1.dpad_down){
            robot.cap.lower(.75);
        }
        else{
            robot.cap.stop();
        }
        if(toggleAllowed(gamepad1.x, x)){
            robot.cap.setZero(DcMotor.RunMode.RUN_USING_ENCODER, 1000, 0);
        }
    }

    public void testableAuto(double heading){




        switch(autoState){
            case 0: //drive forward and backward 2 meters, increasing kp until we observe overshoot
                robot.resetMotors(true);
                autoState++;
                break;
            case 1:
                if(robot.DriveIMUDistance(testableDouble,.4, heading,  testableDirection, 2)){
                    Log.i("Test DriveIMU: ",String.valueOf(testableDouble));
                    testableDouble += .001;
                    testableDirection = !testableDirection;
                    robot.resetMotors(true);
                }
                // will loop here until manually interrupted.
                break;
            case 2:
                if(System.nanoTime() > autoTimer) autoState++;
                break;
            case 3:
                if(robot.driveForward(false, 2, .5)){
                    robot.resetMotors(true);
                    autoTimer = System.nanoTime() + (long)1e9;
                    autoState++;
                }
                break;
        }
    }



    boolean toggleAllowed(boolean button, int buttonIndex)
    {

        /*button indexes:
        0  = a
        1  = b
        2  = x
        3  = y
        4  = dpad_down
        5  = dpad_up
        6  = dpad_left
        7  = dpad_right
        8  = left bumper
        9  = right bumper
        10 = start button
        */

        if (button) {
            if (!buttonSavedStates[buttonIndex])  { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            }
            //       else if(buttonCurrentState[buttonIndex] == buttonSavedStates[buttonIndex] && buttonCurrentState[buttonIndex]){
            else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }




    public String getAlliance(){
        if(isBlue)
            return "Blue";
        return "Red";
    }

    public String autoRun(){
        if(runAutonomous)
            return "Auto will run";
        return "Auto will not run";
    }
    public String getTeleopMode(){
        if(capMode){
            return "Cap Mode";
        }
        return "Particle Mode";
    }
    public double ServoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on 0 - 1 scale
    }




    void configureDashboard() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            }
        });


        telemetry.addLine()
                .addData("active", new Func<String>() {
                    @Override public String value() {
                        return Boolean.toString(active);
                    }
                })
                .addData("state", new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(state);
                    }
                })
                .addData("", new Func<String>() {
                    @Override public String value() {
                        return getTeleopMode();
                    }
                });
//                .addData("Flywheel Speed", new Func<String>() {
//                    @Override public String value() {
//
//                        return Float.toString(robot.particle.flywheelSpeed);
//                    }
//                });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getHeading());
                    }
                })
                .addData("vuAngle", new Func<String>() {
                    @Override public String value() {
                        //return formatAngle(angles.angleUnit, angles.firstAngle);
                        return Double.toString(robot.getVuAngle());
                    }
                })
                .addData("headingRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);

                    }
                })
                .addData("headingOffset", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.offsetHeading);

                    }
                })

                .addData("rollRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitchRaw", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("State", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(autoState);
                    }
                })
                .addData("TicksFL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.motorFrontLeft.getCurrentPosition());
                    }
                })
                .addData("TicksBL", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.motorBackLeft.getCurrentPosition());
                    }
                })
                .addData("TicksAvg", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.getAverageTicks());
                    }
                });
        telemetry.addLine()

                .addData("PID Calc", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.performPID() );
                    }
                })
                .addData("PID Err", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(robot.drivePID.getError());
                    }
                });

        telemetry.addLine()
                .addData("DistRear", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistAft);
                    }
                })
                .addData("ballColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.particle.getBallColor());
                    }
                })
                .addData("DistFore", new Func<String>() {
                    @Override public String value() {
                        return String.valueOf(robot.beaconDistFore);
                    }
                })
                .addData("ForeColor", new Func<String>() {
                    @Override public String value() {
                        return Long.toString(robot.beaconColor);
                    }
                });

//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        //telemetry.addData("Status", "State: " + autoState);
//        //telemetry.addData("Status", "Front Left Ticks: " + Long.toString(motorFrontLeft.getCurrentPosition()));
//        //telemetry.addData("Status", "Average Ticks: " + Long.toString(getAverageTicks()));
//        telemetry.addLine().addData("Normal", beaconPresentRear.getLightDetected());
//
//        telemetry.addLine().addData("ColorFore", beaconColorCache[0] & 0xFF);
//        telemetry.addData("ColorRear", ballColorCache[0] & 0xFF);

    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    long futureTime(float seconds){
        return System.nanoTime() + (long) (seconds * 1e9);
    }
}
