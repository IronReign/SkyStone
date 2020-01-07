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
package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.vision.GoldPos;

import static org.firstinspires.ftc.teamcode.util.Conversions.notdeadzone;


/**
 * This file contains the code for Iron Reign's main OpMode, used for both TeleOp and Autonomous.
 */

@TeleOp(name = "Skystone_6832", group = "Challenge")  // @Autonomous(...) is the other common choice
//  @Autonomous
public class Skystone_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseSkystone.RobotType currentBot = PoseSkystone.RobotType.TomBot;

    private PoseSkystone robot;

    private Autonomous auto;

    private boolean active = true;
    private boolean joystickDriveStarted = false;

    private int state = 0;
    private boolean isBlue = false;

    //loop time profile
    long lastLoopClockTime;
    double loopAvg = 0;
    private static final double loopWeight = .1;

    //drive train control variables
    private double pwrDamper = 1;
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;
    private double pwrFwdL = 0;
    private double pwrStfL = 0;
    private double pwrFwdR = 0;
    private double pwrStfR = 0;
    private double beaterDamper = .75;
    private boolean enableTank = false;
    private boolean bypassJoysticks = false;
    private long damperTimer = 0;
    private int direction = 1;  //-1 to reverse direction
    private int currTarget = 0;

    //sensors/sensing-related variables
    private Orientation angles;

    //these are meant as short term testing variables, don't expect their usage
    //to be consistent across development sessions
    //private double testableDouble = robot.kpDrive;
    private double testableHeading = 0;
    private boolean testableDirection = true;

    //values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[16];
    private int a = 0; //lower glyph lift
    private int b = 1; //toggle grip/release on glyph
    private int x = 2; //no function
    private int y = 3; //raise glyph lift
    private int dpad_down = 4; //enable/disable ftcdash telemetry
    private int dpad_up = 5; //vision init/de-init
    private int dpad_left = 6; //vision provider switch
    private int dpad_right = 7; //switch viewpoint
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)
    private int left_trigger = 11; //vision detection
    private int right_trigger = 12;
    private int back_button = 13;
    private int left_stick_button = 14;
    private int right_stick_button = 15; //sound player

    //values associated with the buttons in the toggleAllowedGP2 method
    private boolean[] buttonSavedStates2 = new boolean[16];

    boolean debugTelemetry = false;

    int stateLatched = -1;
    int stateIntake = -1;
    int stateDelatch = -1;
    boolean isIntakeClosed = true;
    boolean isHooked = false;
    boolean enableHookSensors = false;

    //game mode configuration
    private int gameMode = 0;
    private static final int NUM_MODES = 4;
    private static final String[] GAME_MODES = {"REVERSE", "ENDGAME", "PRE-GAME", "REGULAR"};

    //sound related configuration
    private int soundState = 0;
    private int soundID = -1;

    //auto stuff
    private GoldPos initGoldPosTest;
    private double pCoeff = 0.14;
    private double dCoeff = 1.31;
    private double targetAngle = 287.25;

    private int craneArticulation = 1;

    Telemetry dummyT = new Telemetry() {
        @Override
        public Item addData(String caption, String format, Object... args) {
            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return null;
        }

        @Override
        public boolean removeItem(Item item) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public void clearAll() {

        }

        @Override
        public Object addAction(Runnable action) {
            return null;
        }

        @Override
        public boolean removeAction(Object token) {
            return false;
        }

        @Override
        public boolean update() {
            return false;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            return false;
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {

        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {

        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public Log log() {
            return null;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing "+currentBot+"...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        robot = new PoseSkystone(currentBot);
        robot.init(this.hardwareMap, isBlue);

        auto = new Autonomous(robot, dummyT, gamepad1);


        debugTelemetry = gamepad1.right_trigger > .3;
        debugTelemetry = true;
        if(debugTelemetry) configureDashboardDebug();
        else configureDashboardMatch();
        telemetry.update();

        // waitForStart();
        // this is commented out but left here to document that we are still doing the
        // functions that waitForStart() normally does, but needed to customize it.

        robot.resetMotors(true);
        auto.visionProviderFinalized = false;


        while (!isStarted()) {    // Wait for the game to start (driver presses PLAY)
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            stateSwitch();


            //reset the elbow, lift and supermanLeft motors - operator must make sure robot is in the stowed position, flat on the ground
            if (toggleAllowed(gamepad1.b, b,1)) {
                robot.crane.toggleSwivel();

                if (gamepad1.right_trigger < 0.8) { //unless right trigger is being held very hard, encoders and heading are reset
                    robot.resetEncoders();
                    robot.setZeroHeading();
                    robot.setAutonomousIMUOffset(0); //against lander
                }
                //robot.crane.toggleGripper();
                //servosetup
            }

            if (toggleAllowed(gamepad1.x, x,1   )) {
                isHooked = !isHooked;
                if (isHooked)
                    robot.crane.hookOff();
                else
                    robot.crane.hookOn();
            }

            if (toggleAllowed(gamepad1.y, y,1)) {
                auto.autoDelay++;
                if (auto.autoDelay > 20) auto.autoDelay = 0;
            }

            if (toggleAllowed(gamepad1.left_stick_button, left_stick_button,1))
                enableHookSensors = !enableHookSensors;

            //if (enableHookSensors && robot.distLeft.getDistance(DistanceUnit.METER) < .08)
                //robot.crane.hookOn();
            //if (enableHookSensors && robot.distRight.getDistance(DistanceUnit.METER) < .08)
                //robot.crane.hookOff();

            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_left, dpad_left,1)) {
                auto.visionProviderState = (auto.visionProviderState + 1) % auto.visionProviders.length; //switch vision provider
            }
            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up,1)) {
                auto.initVisionProvider(); //this is blocking
            } else if (auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up,1)) {
                auto.deinitVisionProvider(); //also blocking, but should be very quick
            }
            if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down,1)) {
                auto.enableTelemetry = !auto.enableTelemetry; //enable/disable FtcDashboard telemetry
//                CenterOfGravityCalculator.drawRobotDiagram = !CenterOfGravityCalculator.drawRobotDiagram;
            }
            if (auto.visionProviderFinalized && gamepad1.left_trigger > 0.3) {
                GoldPos gp = auto.vp.detect();
                if (gp != GoldPos.HOLD_STATE)
                    initGoldPosTest = gp;
                telemetry.addData("Vision", "Prep detection: %s%s", initGoldPosTest, gp == GoldPos.HOLD_STATE ? " (HOLD_STATE)" : "");
            }

            if (soundState == 0 && toggleAllowed(gamepad1.right_stick_button, right_stick_button,1)) {
                initialization_initSound();
            }


//            telemetry.addData("Vision", "Backend: %s (%s)", auto.visionProviders[auto.visionProviderState].getSimpleName(), auto.visionProviderFinalized ? "finalized" : System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  ");
//            telemetry.addData("Vision", "FtcDashboard Telemetry: %s", auto.enableTelemetry ? "Enabled" : "Disabled");
//            telemetry.addData("Vision", "Viewpoint: %s", auto.viewpoint);
//
//            telemetry.addData("Sound", soundState == 0 ? "off" : soundState == 1 ? "on" : soundState == 2 ? "file not found" : "other");
//
//            telemetry.addData("Status", "Initialized");
//            telemetry.addData("Status", "Auto Delay: " + Integer.toString((int) auto.autoDelay) + "seconds");
//            telemetry.addData("Status", "Side: " + getAlliance());
//            telemetry.addData("Status", "Hook sensors: " + enableHookSensors);
//            telemetry.addData("Status","hook encoder val: " + robot.crane.hook.getCurrentPosition());
//            telemetry.addData("Turret", "Turret Position raw: " + robot.turret.getCurrentRotationEncoderRaw());

            telemetry.update();

            robot.ledSystem.setColor(LEDSystem.Color.GAME_OVER);

            robot.updateSensors(active);



            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }

        if (auto.vp == null) {
            auto.initDummyVisionProvider(); //this is blocking
        }

        auto.vp.reset();


        robot.crane.restart(.4, .5);

        lastLoopClockTime = System.nanoTime();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stateSwitch();
            if (active) {
                switch (state) {
                    case 0: //code for tele-op control
                        joystickDrive();
                        break;
                    case 1: //autonomous that goes to opponent's crater
                        if (auto.autoSkyStoneRetrieve.execute()) {if(auto.redAutoFull.execute()) active = false;} //active = false;
                        break;
                    case 2: //autonomous that only samples
                        if (auto.redAutoFull.execute()) active = false;
                        //if (auto.walkOfShame.execute()) active = false;
                        break;
                    case 3: //autonomous that starts in our crater
                        if (auto.autoSkyStoneRetrieve.execute()) active = false;
                        break;
                    case 4:
                        if (auto.craterSide_cycle.execute()) active = false;
                        break;
                    case 5:
                        if (auto.depotSide_deposit.execute()) active = false;
                        break;
                    case 6:
//                        if(driveStraight()) active = false;
//                        if(toggleAllowed(gamepad1.right_bumper,right_bumper)){
//                            robot.setForwardTPM(robot.getForwardTPM()+10);
//                        }else if(toggleAllowed(gamepad1.left_bumper,left_bumper)){
//                            robot.setForwardTPM(robot.getForwardTPM()-10);
//                        }
                        turnTest();


                        break;
                    case 7:
                        tpmtuning();
                        break;
                    case 8: //turn to IMU
                        demo();
                        break;
                    case 9:
//                        if (auto.craterSide_extend_reverse.execute()) active = false;
                        ledTest();
                        break;
                    case 10:
//                        if (auto.depotSide_worlds.execute()) active = false;
                        servoTest();
                        break;
                    default:
                        robot.stopAll();
                        break;
                }
                robot.updateSensors(active);
            } else {
                robot.stopAll();
            }

            long loopClockTime = System.nanoTime();
            long loopTime = loopClockTime - lastLoopClockTime;
            if (loopAvg == 0)
                loopAvg = loopTime;
            else
                loopAvg = loopWeight*loopTime + (1-loopWeight)*loopAvg;
            lastLoopClockTime = loopClockTime;

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public boolean driveStraight(){
        return robot.driveForward(true,1,.5);
    }

    int tpmtuningstage = 0;
    public void tpmtuning(){

        switch (tpmtuningstage){
            case 0: //todo - this probably needs work to setup the basic articulation for odometer distance tuning
//                if(robot.goToPosition(0,robot.crane.pos_reverseSafeDrive,.75,.3)){
//                }

                if(toggleAllowed(gamepad1.y,y,1)){
                    robot.resetMotors(true);
                }

                if(toggleAllowed(gamepad1.a,a,1 )){
                    tpmtuningstage++;
                }
                break;
            case 1:
                if(robot.driveForward(true,2,.35)){ //calibrate forward/backward
                //if(robot.driveStrafe(true,2,.35)){ //calibrate strafe if capable - uncomment only one of these at a time
                    tpmtuningstage = 0;
                    robot.resetMotors(true);
                }
                break;
        }
    }



    private void initialization_initSound() {
        telemetry.addData("Please wait", "Initializing Sound");
        //telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        soundID = hardwareMap.appContext.getResources().getIdentifier("gracious", "raw", hardwareMap.appContext.getPackageName());
        boolean success = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        if (success)
            soundState = 1;
        else
            soundState = 2;
    }



    private void demo() {
        if (gamepad1.x)
            robot.maintainHeading(gamepad1.x);
//        if (gamepad1.y) {
        //robot.turret.maintainHeadingTurret(gamepad1.y);
        robot.restaccDemo(toggleAllowed(gamepad1.a,a,1));
//        }
//        if (!gamepad1.y){
//            robot.turret.setPower(0);
//        }
        if (gamepad1.dpad_up) {
            robot.articulate(PoseSkystone.Articulation.manual);
            robot.crane.increaseElbowAngle();
        }
        if (gamepad1.dpad_right) {
            robot.articulate(PoseSkystone.Articulation.manual);
            robot.crane.increaseElbowAngle();
        }
        if (gamepad1.dpad_left) {
            robot.articulate(PoseSkystone.Articulation.manual);
            robot.crane.retractBelt();
        }
        if(toggleAllowed(gamepad2.x,x,2)) {
            robot.crane.setTowerHeight(1);
        }
        if(toggleAllowed(gamepad2.y,y,2)) {
            robot.crane.setTowerHeight(-1);
        }

    }

    int reverse = 1;
    private void joystickDrive() {

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            robot.setAutonSingleStep(true);
            isHooked = false;
            joystickDriveStarted = true;
        }

        if(robot.getArticulation() == PoseSkystone.Articulation.intake){
            reverse = -1;
        }else if(robot.getArticulation() != PoseSkystone.Articulation.intake && robot.getArticulation() != PoseSkystone.Articulation.manual){
            reverse = 1;
        }

        reverse=-1;

        pwrFwd = reverse*direction * pwrDamper * gamepad1.left_stick_y;
        pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;

        pwrFwdL = direction * pwrDamper * gamepad1.left_stick_y;
        pwrStfL = direction * pwrDamper * gamepad1.left_stick_x;

        pwrFwdR = direction * pwrDamper * gamepad1.right_stick_y;
        pwrStfR = direction * pwrDamper * gamepad1.right_stick_x;

        pwrDamper = .70;


        robot.driveMixerDiffSteer(pwrFwd*pwrDamper, pwrRot*pwrDamper);
        //robot.driveDiffTankField(pwrFwdL*pwrDamper, pwrStfL*pwrDamper);
        //robot.driveMixerDiffSteer(0, pwrDamper*pwrStfR);
/*
        if (gamepad1.dpad_right) {
            robot.turret.setTurntableAngle(90);
        }
        if (gamepad1.dpad_left) {
            robot.turret.setTurntableAngle(270);
        }
        if (gamepad1.dpad_up) {
            robot.turret.setTurntableAngle(0);
        }
        if (gamepad1.dpad_down) {
            robot.turret.setTurntableAngle(180);
        }
*/





        //crane controls
//        if (gamepad1.dpad_right) {
//            robot.crane.increaseElbowAngle();
//        }
//        if (gamepad1.dpad_left) {
//            robot.crane.decreaseElbowAngle();
//        }
//        if (gamepad1.dpad_up) {
//            robot.crane.extendBelt();
//        }
//        if (gamepad1.dpad_down) {
//            robot.crane.retractBelt();
//        }

        //turret controls
        if(notdeadzone(gamepad2.right_trigger))
                robot.turret.rotateRight(gamepad2.right_trigger * 5);
        if(notdeadzone(gamepad2.left_trigger))
                robot.turret.rotateLeft(gamepad2.left_trigger * 5);

        if(notdeadzone(gamepad2.right_stick_x) )
            robot.turret.rotateRight(gamepad2.right_stick_x * 5);

        if(toggleAllowed(gamepad1.right_bumper,right_bumper,1)){
                robot.turret.rotateCardinal(true);
        }

        if(toggleAllowed(gamepad1.left_bumper,left_bumper,1)){
            robot.turret.rotateCardinal(false);
        }
        if(toggleAllowed(gamepad1.x,x,1)){

        }
        if(toggleAllowed(gamepad1.b,b,1)){
            robot.crane.extendToPosition(1500,1.0,20);
        }
        if(toggleAllowed(gamepad1.a,a,1)){
//            if(robot.crane.getCurrentAngle() < 20)
//                    robot.articulate(PoseSkystone.Articulation.retrieving);
//            else
                robot.articulate(PoseSkystone.Articulation.retractFromTower);
        }
        if(toggleAllowed(gamepad1.y,y,1)){
            robot.crane.toggleGripper();
        }
        if(toggleAllowed(gamepad2.y,y,2)) {
            robot.crane.setTowerHeight(1);
        }
        if(toggleAllowed(gamepad2.x,x,2)) {
            robot.crane.setTowerHeight(-1);
        }
        if(toggleAllowed(gamepad2.a,a,2)) {
                robot.crane.extendToTowerHeight();
        }
        if(gamepad2.left_bumper) {
            robot.crane.swivelGripper(false);
        }
        if(gamepad2.right_bumper) {
            robot.crane.swivelGripper(true);
        }
        if(toggleAllowed(gamepad2.b,b,2)) {
            robot.crane.toggleSwivel();
        }
        if (toggleAllowed(gamepad2.dpad_right,dpad_right,2)) {
            robot.crane.toggleGripper();
        }
        if(toggleAllowed(gamepad2.dpad_up,dpad_up,2)){
            robot.crane.hookOn();
        }
        if(toggleAllowed(gamepad2.dpad_down,dpad_down,2)){
            robot.crane.hookOff();
        }

        if (notdeadzone(gamepad2.left_stick_y)) {
            robot.crane.adjustElbowAngle(-gamepad2.left_stick_y);
        }

        if (notdeadzone(gamepad2.right_stick_y)) {
            robot.crane.adjustBelt(-gamepad2.right_stick_y);
        }
        if (notdeadzone(gamepad2.right_stick_x)) {
            robot.turret.adjust(gamepad2.right_stick_x);
        }




        robot.crane.update();
        robot.turret.update(opModeIsActive());
    }

    private void joystickDrivePregameMode() {
        robot.setAutonSingleStep(true); //single step through articulations having to do with deploying

        robot.ledSystem.setColor(LEDSystem.Color.CALM);

        boolean doDelatch = false;
        if (toggleAllowed(gamepad1.b, b,1)) {
            stateDelatch++;
            if (stateDelatch > 2) stateDelatch = 0;
            doDelatch = true;
        }

        if (toggleAllowed(gamepad1.x, x,1)) {
            stateDelatch--;
            if (stateDelatch < 0) stateDelatch = 2;
            doDelatch = true;
        }

        if (doDelatch) {
            switch (stateDelatch) {
                case 0:
                    robot.articulate(PoseSkystone.Articulation.retrieving);
                    break;
                case 1:
                    robot.articulate(PoseSkystone.Articulation.deploying);
                    break;
                case 2:
                    robot.articulate(PoseSkystone.Articulation.deployed);
                    break;
                default:
                    break;
            }
        }
    }

    private void logTurns(double target) {
        telemetry.addData("Error: ", target - robot.getHeading());
        //telemetry.update();
    }

    private void joystickDriveEndgameMode() {

        robot.ledSystem.setColor(LEDSystem.Color.SHOT);

        boolean doLatchStage = false;
        robot.driveMixerDiffSteer(pwrFwd, pwrRot);
        if (toggleAllowed(gamepad1.b, b,1)) { //b advances us through latching stages - todo: we should really be calling a pose.nextLatchStage function
            stateLatched++;
            if (stateLatched > 2) stateLatched = 0;
            doLatchStage = true;
        }

        if (toggleAllowed(gamepad1.x, x,1)) { //x allows us to back out of latching stages
            stateLatched--;
            if (stateLatched < 0) stateLatched = 0;
            doLatchStage = true;
        }

        if (doLatchStage) {
            switch (stateLatched) {
                case 0:
                    robot.articulate(PoseSkystone.Articulation.latchApproach);
                    break;
                case 1:
                    robot.articulate(PoseSkystone.Articulation.latchPrep);
                    break;
                case 2:
                    robot.articulate(PoseSkystone.Articulation.latchSet);
                    break;
            }
        }

        if (toggleAllowed(gamepad1.a, a,1)) {
            isHooked = !isHooked;
        }

//        if (isHooked) {
//            robot.crane.hookOn();
//        } else {
//            robot.crane.hookOff();
//        }
    }

    private void turnTest() {
        if (robot.rotateIMU(90, 3)) {
            telemetry.addData("Angle Error: ", 90 - robot.getHeading());
            telemetry.addData("Final Test Heading: ", robot.getHeading());
            robot.setZeroHeading();
            active = false;
        }
        telemetry.addData("Current Angle: ", robot.getHeading());
        telemetry.addData("Angle Error: ", 90 - robot.getHeading());
    }

    private void joystickDriveRegularMode() {

        robot.ledSystem.setColor(LEDSystem.Color.CALM);

        robot.crane.hookOff();

        boolean doIntake = false;
        robot.driveMixerDiffSteer(pwrFwd, pwrRot);

        if (gamepad1.y) {
//            robot.goToSafeDrive();
//            isIntakeClosed = true;
            robot.crane.grabStone();
        }
        if (toggleAllowed(gamepad1.a, a,1)) {
            //isIntakeClosed = !isIntakeClosed;
            robot.crane.ejectStone();
        }


        if (toggleAllowed(gamepad1.b, b,1)) {
//            stateIntake++;
//            if (stateIntake > 3) stateIntake = 0;
//            doIntake = true;
            //robot.crane.stopGripper();
        }

        if (toggleAllowed(gamepad1.x, x,1)) {
            stateIntake--;
            if (stateIntake < 0) stateIntake = 3;
            doIntake = true;
        }

        if (doIntake) {
            switch (stateIntake) {
                case 0:
                    robot.articulate(PoseSkystone.Articulation.preIntake);
                    isIntakeClosed = true;
                    break;
                case 1:
                    robot.articulate(PoseSkystone.Articulation.intake);
                    isIntakeClosed = true;
                    break;
                case 2:
                    robot.articulate(PoseSkystone.Articulation.deposit);
                    break;
                case 3:
                    robot.articulate(PoseSkystone.Articulation.driving);
                    isIntakeClosed = true;
            }
        }


        /*if (isIntakeClosed) {
            robot.crane.closeGate();
        } else {
            robot.crane.grabStone();
        }*/
    }

    private void joystickDriveRegularModeReverse() {

        robot.ledSystem.setColor(LEDSystem.Color.PARTY_MODE_SMOOTH);

        robot.crane.hookOff();

        boolean doIntake = false;


        if (gamepad1.y) {
            robot.articulate(PoseSkystone.Articulation.reverseDriving);
            isIntakeClosed = true;
        }
        if (toggleAllowed(gamepad1.a, a,1)) {
            isIntakeClosed = !isIntakeClosed;
        }


        if (toggleAllowed(gamepad1.b, b,1)) {
            stateIntake++;
            if (stateIntake > 3) stateIntake = 0;
            doIntake = true;
        }

        if (toggleAllowed(gamepad1.x, x,1)) {
            stateIntake--;
            if (stateIntake < 0) stateIntake = 3;
            doIntake = true;
        }

        if (doIntake) {
            switch (stateIntake) {
                case 0:
                    robot.articulate(PoseSkystone.Articulation.reverseIntake);
                    pwrRot-=.25;
                    //robot.crane.setBeltToElbowModeEnabled();
                    isIntakeClosed = true;
                    break;
                case 1:
                    robot.articulate(PoseSkystone.Articulation.prereversedeposit);
                    //robot.crane.setBeltToElbowModeDisabled();
                    isIntakeClosed = true;
                    break;
                case 2:
                    robot.articulate(PoseSkystone.Articulation.reverseDeposit);
                    //robot.crane.setBeltToElbowModeDisabled();
                    break;
                case 3:
                    robot.articulate(PoseSkystone.Articulation.reverseDriving);
                    //robot.crane.setBeltToElbowModeDisabled();
                    isIntakeClosed = true;
                    break;
            }
        }
        robot.driveMixerDiffSteer(pwrFwd, pwrRot);

        /*if (isIntakeClosed) {
            robot.crane.closeGate();
        } else {
            robot.crane.grabStone();
        }*/
    }


    //the method that controls the main state of the robot; must be called in the main loop outside of the main switch
    private void stateSwitch() {
        if (!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper,1)) {

                state--;
                if (state < 0) {
                    state = 10;
                }
                robot.resetMotors(true);
                active = false;
            }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper,1)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                robot.resetMotors(true);
                active = false;
            }

        }

        if (toggleAllowed(gamepad1.start, startBtn,1)) {
            robot.resetMotors(true);
            active = !active;
        }
    }


    //checks to see if a specific button should allow a toggle at any given time; needs a rework
    private boolean toggleAllowed(boolean button, int buttonIndex, int gpId) {
        if (button) {
            if(gpId == 1) {
                if (!buttonSavedStates[buttonIndex]) { //we just pushed the button, and when we last looked at it, it was not pressed
                    buttonSavedStates[buttonIndex] = true;
                    return true;
                } else { //the button is pressed, but it was last time too - so ignore

                    return false;
                }
            }
            else{
                if (!buttonSavedStates2[buttonIndex]) { //we just pushed the button, and when we last looked at it, it was not pressed
                    buttonSavedStates2[buttonIndex] = true;
                    return true;
                } else { //the button is pressed, but it was last time too - so ignore

                    return false;
                }
            }
        }
        if(gpId == 1)
        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        else
        buttonSavedStates2[buttonIndex] = false;
        return false; //not pressed

    }


    private String getAlliance() {
        if (isBlue)
            return "Blue";
        return "Red";
    }


    private void configureDashboardDebug() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        //telemetry.addAction(() ->
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                //angles = robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX)

       // );

        telemetry.addLine()
                .addData("active", () -> active)
                .addData("state", () -> state)
                .addData("autoStage", () -> auto.autoStage)
                .addData("Game Mode", () -> GAME_MODES[gameMode])
                .addData("Articulation", () -> robot.getArticulation());
        telemetry.addLine()
                //.addData("elbowA", () -> robot.crane.isActive())
                .addData("elbowC", () -> robot.crane.getElbowCurrentPos())
                .addData("elbowT", () -> robot.crane.getElbowTargetPos());
        telemetry.addLine()
                .addData("liftPos", () -> robot.crane.getExtendABobCurrentPos());
        telemetry.addLine()
                .addData("roll", () -> robot.getRoll())
                .addData("pitch", () -> robot.getPitch())
                .addData("heading", () -> robot.getHeading())
                .addData("yawraw", () -> robot.getHeading());
        telemetry.addLine()
                .addData("Loop time", "%.0fms", () -> loopAvg/1000000)
                .addData("Loop time", "%.0fHz", () -> 1000000000/loopAvg);        //telemetry.addLine()w
        telemetry.addLine()
                .addData("Turret Pos", ()-> robot.turret.getCurrentRotationEncoderRaw())
                .addData("Turret Target",()-> robot.turret.getTargetRotationTicks());
        telemetry.addLine()
                .addData("Turret Heading", ()-> robot.turret.getHeading())
                .addData("Turret Target`s", ()->robot.turret.getTurretTargetHeading());
        telemetry.addLine()
                .addData("Turret Current tower height: ", ()-> robot.crane.getCurrentTowerHeight());
        telemetry.addLine()
                .addData("Turret Current angle ", ()-> robot.turret.getHeading())
                .addData("Joystick Y ", ()-> gamepad1.right_stick_y);

        // .addData("calib", () -> robot.imu.getCalibrationStatus().toString());
        //telemetry.addLine()
                //.addData("drivedistance", () -> robot.getAverageAbsTicks());
        //telemetry.addLine()
                //.addData("status", () -> robot.imu.getSystemStatus().toShortString())
                //.addData("mineralState", () -> auto.mineralState)
//                .addData("distForward", () -> robot.distForward.getDistance(DistanceUnit.METER))
//                .addData("distLeft", () -> robot.distLeft.getDistance(DistanceUnit.METER))
//                .addData("distRight", () -> robot.distRight.getDistance(DistanceUnit.METER))
                //.addData("depositDriveDistaFnce", () -> robot.depositDriveDistance);




    }

    private void configureDashboardMatch() {
        // Configure the dashboard.

        telemetry.addLine()
                .addData("active", () -> active)
                .addData("state", () -> state)
                .addData("Game Mode", () -> GAME_MODES[gameMode])
                .addData("Articulation", () -> robot.getArticulation());

        telemetry.addLine()
                .addData("Loop time", "%.0fms", () -> loopAvg / 1000000)
                .addData("Loop time", "%.0fHz", () -> 1000000000 / loopAvg);

    }


    private int servoTest = 1005;

    private void servoTest() {
        //robot.ledSystem.movement.setPosition(Conversions.servoNormalize(servoTest));
        if (toggleAllowed(gamepad1.a, a,1))
            servoTest -= 10;
        else if (toggleAllowed(gamepad1.y, y,1))
            servoTest += 10;
        telemetry.addData("Pulse width", servoTest);
    }

    private void ledTest() {
        int idx = (int) ((System.currentTimeMillis() / 2000) % LEDSystem.Color.values().length);
        robot.ledSystem.setColor(LEDSystem.Color.values()[idx]);
        telemetry.addData("Color", LEDSystem.Color.values()[idx].name());
    }

}
