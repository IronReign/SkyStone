package org.firstinspires.ftc.teamcode.robots.tombot;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystoneGripPipeline;
import org.firstinspires.ftc.teamcode.vision.StonePos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersRoverRuckus;
import org.firstinspires.ftc.teamcode.vision.VisionProviderSkystoneByMaheshMaybe;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/**
 * Class to keep all autonomous-related functions and state-machines in
 */
public class Autonomous {

    private PoseSkystone robot;
    private Telemetry telemetry;
    private Gamepad gamepad1;

    //vision-related configuration
    public VisionProvider vp;
    public VisionProviderSkystoneByMaheshMaybe vps;
    public int visionProviderState;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProvidersRoverRuckus.visionProviders;
    public static final Viewpoint viewpoint = Viewpoint.WEBCAM;
    public int mineralState = 1;
    private MineralStateProvider mineralStateProvider = () -> mineralState;

    //staging and timer variables
    public float autoDelay = 0;
    public Stage autoStage = new Stage();
    public Stage autoSetupStage = new Stage();

    //auto constants
    private static final double DRIVE_POWER = .65;
    private static final float TURN_TIME = 2;
    private static final float DUCKY_TIME = 1.0f;

    public Autonomous(PoseSkystone robot, Telemetry telemetry, Gamepad gamepad1) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
    }


    private boolean sample() {
        Mat output = robot.pipeline.process();

        if(!(output == null)) {
            Bitmap bm = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(output, bm);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Position", robot.pipeline.info.toString());

            robot.dashboard.sendImage(bm);
            robot.dashboard.sendTelemetryPacket(packet);
        } else {
            telemetry.addData("Status", "output is null");
            telemetry.update();
        }

        int cycles = 0; //take this out eventually
        StonePos quarryPosition = robot.pipeline.info.getQuarryPosition();
        if(cycles < 10000) {
            if (!quarryPosition.equals(StonePos.NONE_FOUND)) {
                switch (quarryPosition) {
                    case SOUTH:
                        mineralState = 0;
                        break;
                    case MIDDLE:
                        mineralState = 1;
                        break;
                    case NORTH:
                        mineralState = 2;
                        break;
                    default:
                        mineralState = 1;
                        telemetry.update();
                        break;
                }
                return true;
            } else {
                cycles++;
                return false;
            }
        }
        else {
            mineralState = 1;
            cycles = 0;
            return true;
        }
    }

//    public StateMachine visionTest = getStateMachine(autoStage)
//            .addState(() -> {
//             robot.xPos = robot.vps.detect();
//             return false;
//            })
//            .build();


    public StateMachine simultaneousStateTest = getStateMachine(autoStage)
            .addSimultaneousStates(
                    () -> {robot.turret.rotateRight(0.25); return false;},
                    () -> {robot.driveMixerDiffSteer(0.25, 0.25); return false;}
            )
            .build();

    public StateMachine retractFromTower = getStateMachine(autoStage)
            .addSimultaneousStates(
                    () -> robot.crane.toggleGripper(),
                    () -> robot.crane.setElbowTargetAngle(robot.crane.getCurrentAngle() + 15),
                    () -> {robot.articulate(PoseSkystone.Articulation.retriving2); return true; },
                    () -> {robot.articulate(PoseSkystone.Articulation.cardinalBaseLeft); return true; }
            ).build();

    public StateMachine visionTest = getStateMachine(autoStage)
            .addState(() ->  {
                Mat output = robot.pipeline.process();

                if(!(output == null)) {
                    Bitmap bm = Bitmap.createBitmap(output.width(), output.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(output, bm);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Position", robot.pipeline.info.toString());

                    robot.dashboard.sendImage(bm);
                    robot.dashboard.sendTelemetryPacket(packet);

                    telemetry.addData("vision shite", () -> robot.pipeline.info.getQuarryPosition());
                }
                return false;
            }).build();

    public StateMachine AutoFull = getStateMachine(autoStage)
            //open and align gripper for 1st skystone
            .addState(() -> (robot.crane.setElbowTargetPos(500,1)))
            .addState(() -> {robot.pipeline.setIsBlue(!robot.isBlue); return true;})
            .addState(() -> sample())
            .addState(() -> robot.crane.toggleGripper())
            .addState(() -> robot.crane.setGripperSwivelRotation(1630))
            .addState(() -> (robot.crane.setElbowTargetPos(300,1)))


            //adjust turret if needed to point to correct stone
            .addMineralState(mineralStateProvider,
                    () -> robot.turret.rotateIMUTurret(260,2),
                    () -> true,
                    () -> robot.turret.rotateIMUTurret(285,2))

            .addMineralState(mineralStateProvider,
                    () -> robot.crane.setGripperSwivelRotation(1450),
                    () -> true,
                    () -> robot.crane.setGripperSwivelRotation(1700))

            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(2190,1,130),
                    () -> robot.crane.extendToPosition(2190,1,120),
                    () -> robot.crane.extendToPosition(2190,1,120))

            //drop and snap gripper
            .addState(() ->robot.crane.setElbowTargetPos(-10,1))

            //retrieve stone
            .addState(() ->robot.crane.setElbowTargetPos(30,1))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromBlockAuton))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //pull away from wall half a meter
            .addState(() -> (robot.isBlue ?
                    robot.driveIMUDistance(.6,90,true,.470) :
                    robot.driveIMUDistance(.6,270,true,.470))
            )//this and ^^^^ put the robot in front of the build plate
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //rotate north
            .addState(() -> (robot.rotateIMU(0.0, 4)))

            //drive to foundation
            .addState(() -> (robot.driveIMUDistance(.6,0.0,true,1.95)))
            .addState(() -> (robot.crane.setElbowTargetPosWithSlop(200,50,1)))

            //deposit stone
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.turnTurretToBaseAuton))
            .addState(() ->robot.crane.extendToPosition(1050,1,60))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

//            //drive south to next stone
//            .addState(() -> (robot.goToBlock(2)))
//
//            //position elbow, arm and gripper for oblique pickup
//            .addState(() -> (robot.crane.setElbowTargetPos(220,1)))
//            .addState(() -> robot.turret.rotateIMUTurret(225,3))//deposit stone
//            .addState(() -> robot.crane.setGripperSwivelRotation(1200))
//            .addState(() ->robot.crane.extendToPosition(930,1,30))
//
//
//            //grab stone
//            .addState(() -> (robot.crane.setElbowTargetPos(0,1)))
//            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
//
//            //grab stone
//            .addState(() -> (robot.crane.setElbowTargetPos(400,1)))
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromBlockAuton))

//            //return to foundation with 2nd stone todo: not tested yet
//            .addState(() -> (robot.StoneToFoundation(nextAutonStone(1))))
//
//            //slam duncc
//            .addState(() -> (robot.crane.setElbowTargetPos(300,1)))
//            .addState(() -> robot.turret.rotateIMUTurret(270,3))
//            .addState(() -> robot.crane.setGripperSwivelRotation(1600))
//            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
//            .addState(() ->robot.crane.extendToPosition(550,.7,10))
//            .addState(() -> (robot.crane.setElbowTargetPos(80,1)))
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))

            //drive to and hook onto foundation
            .addSingleState(() -> robot.crane.hookOff()) //makes sure the hook is up properly
            .addState(() -> (robot.driveIMUUntilDistance(.3,0,true,.35)))
            .addTimedState(.4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.turnToBaseAuton))
            .addState(() -> (robot.driveForward(true, robot.getDistForwardDist()+.07, .30)))
            .addTimedState(.4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.crane.hookOn())

            //backup and try and turn
            .addState(() -> (robot.driveIMUDistance(.6,340,false,1)))
            .addState(() -> (robot.driveIMUDistance(.5,0.0,true,.45)))

            //hook off and drive back into the bridge
            .addSingleState(() -> robot.crane.hookOff())
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.driveIMUDistance(.4,45.0,false,1)))
            .build();


    public StateMachine walkOfShamePointsSouth = getStateMachine(autoStage)
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(0.25))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.crane.setElbowTargetPos(robot.crane.elbowMin, 1))
            .addState(() -> robot.turret.rotateIMUTurret(180.0, 6))
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(0.5))
            .build();


    public StateMachine walkOfShamePointNorth = getStateMachine(autoStage)
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(0.25))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> robot.crane.setElbowTargetPos(robot.crane.elbowMin, 1))
            .addState(() -> robot.turret.rotateIMUTurret(0.0, 6))
            .addSingleState(() -> robot.crane.setExtendABobLengthMeters(0.5))
            .build();

    public StateMachine autoMethodTesterTool = getStateMachine(autoStage) // I do actually use this, do not delete

            .build();





    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                  Old Autonomous Routines                                   //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> robot.crane.setGripperPos(robot.crane.toggleGripper()))  //resetMotors(true)
                .stateEndAction(() -> robot.turret.maintainHeadingTurret(false))
                .stage(stage);
    }

    public void deinitVisionProvider() {
        telemetry.addData("Please wait", "Deinitializing vision");
        //telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        vp.shutdownVision();
        vp = null;
        visionProviderFinalized = false;
    }

    public void initVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            //telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = visionProviders[visionProviderState].newInstance();
            //vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    public void initDummyVisionProvider() {
        try {
            telemetry.addData("Please wait", "Initializing vision");
            //telemetry.update();
            robot.ledSystem.setColor(LEDSystem.Color.CALM);
            vp = VisionProvidersRoverRuckus.defaultProvider.newInstance();
            //vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

    int stoneCount = 0;
    boolean[] quarryStones = new boolean[6];

    public int nextAutonStone(int firstStone){
        if (stoneCount == 0) {
            quarryStones[firstStone] = true;
            stoneCount++;
            return firstStone;
        }
        if (stoneCount == 1) {
            quarryStones[firstStone+3] = true;
            stoneCount++;
            return firstStone+3;
        }
        if (stoneCount>1 && stoneCount <6){
            for (int i = 0; i < quarryStones.length; i++){
                if (!quarryStones[i]) //this is the first stone still false
                {   stoneCount++;
                    quarryStones[i] = true;
                    return i;
                }
            }

        }
        return -1; //this would be a fail
    }
}
