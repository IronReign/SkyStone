package org.firstinspires.ftc.teamcode.robots.ScytheMeUp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystoneVisionProvider;
import org.firstinspires.ftc.teamcode.vision.StonePos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersSkystone;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private Pose robot;
    private Telemetry telemetry;
    private Gamepad gamepad1;

    public static int sampleExtendMiddle = 2210;
    public static int sampleExtendLeft = 2200;
    public static int sampleExtendRight = 2200;
    public static boolean sampleContinue = false;

    // vision-related configuration
    public SkystoneVisionProvider vp;
    public int visionProviderState = 2;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends SkystoneVisionProvider>[] visionProviders = VisionProvidersSkystone.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int skystoneState = 1;
    private MineralStateProvider skystoneStateProvider = () -> skystoneState;

    // staging and timer variables
    public float autoDelay = 0;
    public Stage autoStage = new Stage();
    public Stage autoSetupStage = new Stage();

    // auto constants
    private static final double DRIVE_POWER = .65;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 1.0f;

    public Autonomous(Pose robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }

    public boolean sample() {
        // Turn on camera to see which is gold
        StonePos gp = vp.detectSkystone().getQuarryPosition();
        // Hold state lets us know that we haven't finished looping through detection
        if (gp != StonePos.NONE_FOUND) {
            switch (gp) {
                case SOUTH:
                    skystoneState = 0;
                    break;
                case MIDDLE:
                    skystoneState = 1;
                    break;
                case NORTH:
                    skystoneState = 2;
                    break;
                case NONE_FOUND:
                case ERROR1:
                case ERROR2:
                case ERROR3:
                default:
                    skystoneState = 1;
                    break;
            }
            telemetry.addData("Vision Detection", "StonePos: %s", gp.toString());
            vp.shutdownVision();
            return true;
        } else {
            telemetry.addData("Vision Detection", "NONE_FOUND (still looping through internally)");
            return false;
        }
    }

    // public StateMachine visionTest = getStateMachine(autoStage)
    // .addState(() -> {
    // robot.xPos = robot.vps.detect();
    // return false;
    // })
    // .build();


    public StateMachine AutoFull = getStateMachine(autoStage)
            //just kept this as an example of a complex statemachine
            // open and align gripper for 1st skystone
            /*
            .addSingleState(() -> robot.crane.hookOn()) // makes sure the hook is down properly
            .addState(() -> robot.isBlue ?
                    robot.turret.rotateIMUTurret(90,3) :
                    robot.turret.rotateIMUTurret(270,3))
            .addState(() -> (robot.crane.setElbowTargetPos(600, 1)))
            .addTimedState(3f, () -> sample(),  () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.crane.toggleGripper())
            .addSingleState(() -> robot.crane.hookOff()) // makes sure the hook is down properly
            .addState(() -> robot.crane.setGripperSwivelRotation(robot.crane.swivel_Front-100))
            .addState(() -> (robot.crane.setElbowTargetPos(500, 1)))

            // adjust turret if needed to point to correct stone
            .addMineralState(skystoneStateProvider,
                    () -> robot.turret.rotateIMUTurret(260, 2),
                    () -> true,
                    () -> robot.turret.rotateIMUTurret(285, 2))

            .addMineralState(skystoneStateProvider,
                    () -> robot.crane.setGripperSwivelRotation(1450),
                    () -> true,
                    () -> robot.crane.setGripperSwivelRotation(1700))

            .addMineralState(skystoneStateProvider,
                    () -> robot.crane.extendToPosition(2200, 1),
                    () -> robot.crane.extendToPosition(2210, 1),
                    () -> robot.crane.extendToPosition(2200, 1))

            // drop and snap gripper
            .addState(() -> robot.crane.setElbowTargetPos(30, 1))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            .addState(() -> robot.crane.setElbowTargetPos(30, 1))

            .addSimultaneousStates( //retract arm while driving forward - trying to get about the same speed so stone doesn't really move
                    () -> robot.crane.setElbowTargetPos(30, 1),
                    ()-> robot.crane.extendToPosition(robot.crane.extendMin, 1), // maybe look at using retract articulation
                    //pull away from wall half a meter
                    () -> robot.isBlue ?
                    robot.driveIMUDistanceWithReset(.6,90,true,.470) :
                    robot.driveIMUDistanceWithReset(.6,270,true,.470))

            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            // retrieve stone
            .addSingleState(() -> robot.articulate(Pose.Articulation.retrieveStone))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))


            // rotate north
            .addState(() -> (robot.rotateIMU(0.0, 4)))

            // drive to foundation
            .addState(() -> (robot.driveIMUDistanceWithReset(.6, 0.0, true, 1.95)))
            .addState(() -> (robot.crane.setElbowTargetPos(300, 1)))

            // deposit stone
            .addState(() -> robot.isBlue ?
                    robot.turret.rotateIMUTurret(90,3) :
                    robot.turret.rotateIMUTurret(270,3))
            .addState(() -> robot.crane.extendToPosition(1050, 1))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(Pose.Articulation.retractFromTower))
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))



            .addTimedState(.4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.isBlue ?
                    robot.rotateIMU(90,3) :
                    robot.rotateIMU(270,3))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.isBlue ? //this is to drive and center it todo- is use this ^^^^ line, which should do the job better when distForward gets good
                    robot.driveIMUDistanceWithReset(.3,90,true,.2) :
                    robot.driveIMUDistanceWithReset(.3,270,true,.2))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.crane.hookOn())

            // backup and try and turn
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.isBlue ? //this is to drive and center it todo- is use this ^^^^ line, which should do the job better when distForward gets good
                    robot.driveIMUDistanceWithReset(.3,90,true,.05) :
                    robot.driveIMUDistanceWithReset(.3,270,true,.05))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.driveIMUDistance(1, 290, false, .7)))


            //park
            .addState(() -> (robot.turret.rotateIMUTurret(225,3)))
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(.9))
            */

            .build();

    public StateMachine walkOfShame = getStateMachine(autoStage)
            .addState(() -> robot.crane.setElbowTargetPos(robot.crane.elbowMin-10, 1))
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(0.25))
            .build();

    public StateMachine autoMethodTesterTool = getStateMachine(autoStage) // I do actually use this, do not delete
            //.addSingleState(() -> robot.articulate(Pose.Articulation.autoRotateToFaceStone))
            .build();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // //
    // Old Autonomous Routines //
    // //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
        return
                StateMachine.builder().stateSwitchAction(() -> robot.crane.setGripperPos(robot.crane.toggleGripper())) // resetMotors(true)
                .stateEndAction(() -> robot.maintainHeading(false)).stage(stage);
    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        // telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
             vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint, !robot.isBlue);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            // telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProvidersSkystone.defaultProvider.newInstance();
             vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint, !robot.isBlue);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    int stoneCount = 0;
    boolean[] quarryStones = new boolean[6];

    public int nextAutonStone(int firstStone) {
        if (stoneCount == 0) {
            quarryStones[firstStone] = true;
            stoneCount++;
            return firstStone;
        }
        if (stoneCount == 1) {
            quarryStones[firstStone + 3] = true;
            stoneCount++;
            return firstStone + 3;
        }
        if (stoneCount > 1 && stoneCount < 6) {
            for (int i = 0; i < quarryStones.length; i++) {
                if (!quarryStones[i]) // this is the first stone still false
                {
                    stoneCount++;
                    quarryStones[i] = true;
                    return i;
                }
            }

        }
        return -1; // this would be a fail
    }
}
