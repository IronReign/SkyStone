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
 * adb connect 192.168.43.1:5555
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
        robot.pipeline.process();
        StonePos quarryPosition = robot.pipeline.info.getQuarryPosition();
        if(!quarryPosition.equals(StonePos.NONE_FOUND)) {
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
                    break;
            }
            return true;
        } else
            return false;

    }

//    public StateMachine visionTest = getStateMachine(autoStage)
//            .addState(() -> {
//             robot.xPos = robot.vps.detect();
//             return false;
//            })
//            .build();



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
                }
                return false;
            }).build();

    public StateMachine redAutoFull = getStateMachine(autoStage)
            //open and align gripper for 1st skystone
            .addState(() -> robot.crane.toggleGripper())
            .addState(() -> robot.crane.setGripperSwivelRotation(1600))
            //
            .addState(() -> (robot.crane.setElbowTargetPos(300,1)))

            //adjust turret if needed to point to correct stone
            .addMineralState(mineralStateProvider,
                    () -> { robot.turret.rotateIMUTurret(270+15,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_left_Block);},
                    () -> true,
                    () -> { robot.turret.rotateIMUTurret(270-15,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_Right_Block);})

            //position gripper over
            .addState(() ->robot.crane.extendToPosition(2170,1,120))
            //drop and snap gripper
            .addState(() ->robot.crane.setElbowTargetPos(0,1))

            //retrieve stone
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromBlockAuton))
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //pull away from wall half a meter
            .addState(() -> (robot.driveIMUDistance(.6,270,true,.460)))//this and ^^^^ put the robot in front of the build plate
            .addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //rotate north
            .addState(() -> (robot.rotateIMU(0.0, 4)))

            //drive to foundation
            .addState(() -> (robot.driveIMUDistance(.6,0.0,true,1.95)))
            .addState(() -> (robot.crane.setElbowTargetPos(200,1)))

            //deposit stone
            .addState(() -> robot.turret.rotateIMUTurret(270,3))//deposit stone
            .addState(() ->robot.crane.extendToPosition(700,1,10))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //drive south to next stone
            .addState(() -> (robot.goToBlock(2)))

            //position elbow, arm and gripper for oblique pickup
            .addState(() -> (robot.rotateIMU(0.0, 4)))
            .addState(() -> (robot.crane.setElbowTargetPos(300,1)))
            .addState(() -> robot.turret.rotateIMUTurret(230,3))//deposit stone
            .addState(() -> robot.crane.setGripperSwivelRotation(1200))
            .addState(() ->robot.crane.extendToPosition(1070,1,30))


            //grab stone
            .addState(() -> (robot.crane.setElbowTargetPos(0,1)))

            //grab stone
            .addState(() -> (robot.crane.setElbowTargetPos(400,1)))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromBlockAuton))
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //return to foundation with 2nd stone todo: not tested yet
            .addState(() -> (robot.StoneToFoundation(nextAutonStone(1))))

            //slam duncc
            .addState(() -> (robot.crane.setElbowTargetPos(300,1)))
            .addState(() -> robot.turret.rotateIMUTurret(270,3))
            .addState(() -> robot.crane.setGripperSwivelRotation(1600))
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() ->robot.crane.extendToPosition(550,.7,10))
            .addState(() -> (robot.crane.setElbowTargetPos(80,1)))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))

            //drive to and hook onto foundation
            .addSingleState(() -> robot.crane.hookOff()) //makes sure the hook is up properly
            .addTimedState(.4f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.driveIMUUntilDistance(.3,0,true,.5)))
            .addState(() -> (robot.rotateIMU(270.0, 6)))
            .addState(() -> (robot.driveForward(true, robot.getDistForwardDist(), .30)))
            .addSingleState(() -> robot.crane.hookOn())

            //backup and try and turn
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            //.addState(() -> (robot.driveForward(false, .3, .30)))
            .addState(() -> (robot.rotateIMU(0.0, 10)))
            .addState(() -> (robot.driveIMUDistance(1,0.0,true,.7)))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))

            //hook off and drive back into the bridge
            .addSingleState(() -> robot.crane.hookOff())
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.driveIMUDistance(.4,0.0,false,1)))
            .build();


    public StateMachine walkOfShameLeft = getStateMachine(autoStage)
            .addState(() -> (robot.turret.rotateIMUTurret(robot.turret.getHeading()-90,3)))
            .build();


    public StateMachine walkOfShameRight = getStateMachine(autoStage)
            .addState(() -> (robot.turret.rotateIMUTurret(robot.turret.getHeading()+90,3)))
            .build();

    public StateMachine autoMethodTesterTool = getStateMachine(autoStage)
            .addState(() -> (robot.rotateIMU(90, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(180, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(270, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(0, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(270, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(180, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(90, 4)))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.rotateIMU(0, 4)))
            .build();





    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                            //
    //                                  Old Autonomous Routines                                   //
    //                                                                                            //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public StateMachine autoSetupReverse = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reversedeploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.reversedeployed) //wait until robot articulation in progress
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.reverseDriving) //wait until done
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotateIMU(0, 1)) //turn back to center
            .build();

    public StateMachine depotSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))
            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_worlds = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .02, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMin(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to depot
            .addState(() -> robot.crane.extendToMax(1,10))
            .addState(() -> robot.crane.setElbowTargetPos(robot.crane.autodepotthingy, 1))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin())
            .addState(() -> robot.driveForward(false, .8, DRIVE_POWER))
            .build();

    public StateMachine depotSide_deposit = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .234, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .224, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(36, TURN_TIME),
                    () -> robot.rotateIMU(354, TURN_TIME),
                    () -> robot.rotateIMU(318, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1350, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+900, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,10))
            .addState(() -> {robot.articulate(PoseSkystone.Articulation.reverseDepositAssisted); return robot.rotateIMU(0, 3);})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid-50, 1, 10))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addTimedState(1,
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addSingleState(() -> robot.crane.extendToMin(1,10))
            .addState(() -> robot.rotateIMU(80, 3)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, .8)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 2)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            .build();

    public StateMachine craterSide_cycle = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .020, .40)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(37, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(323, TURN_TIME))

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.stopIntake())
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.crane.ejectStone())
            .addState(() -> robot.driveForward(true, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseIntake))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.crane.extendToMax())
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation. prereversedeposit))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addState(() -> robot.rotateIMU(345, 3))
            .addState(() -> robot.driveForward(false, .1, DRIVE_POWER))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDeposit))
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addTimedState(2,
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.rotateIMU(0, 1)) //turn to crater
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving))
//            .addSingleState(() -> robot.crane.eject())
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseIntake))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addState(() -> robot.crane.extendToMax())
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation. prereversedeposit))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.reverseDeposit))
//            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
//            .addState(() -> robot.rotateIMU(345, 3))
//            .addTimedState(2,
//                    () -> robot.crane.eject(),
//                    () -> robot.crane.stopIntake())
//            .addState(() -> robot.rotateIMU(0, 4)) //turn to crater
            .build();


    public StateMachine autoSetup = getStateMachine(autoSetupStage)
            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.setAutonSingleStep(false)) //turn off autonSingleState
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.RED)) //red color
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.deploying)) //start deploy
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.driving) //wait until done
            .addState(() -> robot.articulate(PoseSkystone.Articulation.driving, true))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE)) //purple color
            .addState(() -> robot.rotateIMU(0, 1)) //turn back to center
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> robot.driveForward(false, .05, DRIVE_POWER)) //move back to see everything
            //.addTimedState(0.5f, () -> {}, () -> {}) //wait for the robot to settle down
            .addState(() -> sample()) //detect the mineral
            .addState(() -> robot.driveForward(true, .05, DRIVE_POWER)) //move forward again
            .build();

    public StateMachine depotSide_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(80, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.3, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(120, 3)) //turn to crater
            .addState(() -> robot.crane.extendToMax(1,10))
            //.addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .build();

    public StateMachine depotSample_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> (robot.driveForward(true, .334, .40)))

            .addState(() -> robot.crane.extendToMax(1,15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMin(1,15))

            .addState(() -> (robot.driveForward(false, .314, .45)))
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10);},
                    () -> { robot.crane.ejectStone(); return robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10);})
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> {robot.crane.stopIntake(); return robot.crane.extendToMid(1,10);})
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            //.addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)

            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .build();

    public StateMachine craterSide_extend_reverse = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.1, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(120, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
//            .addState(() -> robot.crane.extendToMax(1,10))
            .addSingleState(() -> robot.crane.setExtendABobTargetPos(robot.crane.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(34, 0.6))
            .addState(() -> robot.rotateIMU(310, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.crane.extendToMax())
            .build();

    public StateMachine craterSide_extend_reverse_team_marker = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addState(() -> robot.rotateIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(128, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid+300,1,10))//extendToMin(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(90, 4)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.4, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            .addState(() -> robot.crane.extendToPosition(robot.crane.extendMid+400,1,10))
            .build();

    public StateMachine depotSide = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addMineralState(mineralStateProvider, //turn to wall
                    () -> true,
                    () -> robot.rotateIMU(225, 4),
                    () -> robot.rotateIMU(225, 4))
            .addMineralState(mineralStateProvider, //move forward a little
                    () -> true,
                    () -> robot.driveForward(false, .090, DRIVE_POWER),
                    () -> robot.driveForward(false, .160, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(303, 5)) //turn to crater
            .addState(() -> robot.driveForward(false, 1.05, DRIVE_POWER)) //go to crater
            .addState(() -> robot.rotateIMU(310, 1.5)) //turn to crater
            .addState(() -> robot.driveForward(false, .80, DRIVE_POWER)) //go to grater
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done
            .build();

    public StateMachine depotSample = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .604, DRIVE_POWER),
                    () -> robot.driveForward(true, .47, DRIVE_POWER),
                    () -> robot.driveForward(true, .604, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //turn to depot
                    () -> robot.rotateIMU(345, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(15, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to depot
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .762, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .build();

    public StateMachine craterSide_extend = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(270, 3)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(false, 1.43344, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.48988, DRIVE_POWER),
                    () -> robot.driveForward(false, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(310, 3)) //turn to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.extendToMax(1,10))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.driving, true))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(100, 0.6))
            .addState(() -> robot.rotateIMU(130, 3))
            .addState(() -> robot.driveForward(false, .5, DRIVE_POWER))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.preIntake, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))


            /*.addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.eject(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done*/
            .build();

    private boolean resetIMUBool() {
        robot.resetIMU();
        return true;
    }

    public StateMachine craterSide = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetup)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))
            .addMineralState(mineralStateProvider, //move to mineral
                    () -> robot.driveForward(true, .880, DRIVE_POWER),
                    () -> robot.driveForward(true, .70, DRIVE_POWER),
                    () -> robot.driveForward(true, .890, DRIVE_POWER))
            .addMineralState(mineralStateProvider, //move back
                    () -> robot.driveForward(false, .440, DRIVE_POWER),
                    () -> robot.driveForward(false, .35, DRIVE_POWER),
                    () -> robot.driveForward(false, .445, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(80, TURN_TIME)) //turn parallel to minerals
            .addMineralState(mineralStateProvider, //move to wall
                    () -> robot.driveForward(true, 1.4, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.5, DRIVE_POWER),
                    () -> robot.driveForward(true, 1.9, DRIVE_POWER))
            .addState(() -> robot.rotateIMU(135, TURN_TIME)) //turn to depot
            .addState(() -> robot.driveForward(true, 1.2, DRIVE_POWER)) //move to depot
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true)) //so we can start overriding
            .addState(() -> robot.crane.setElbowTargetPos(618, 1))
            .addState(() -> robot.crane.extendToMid(1, 15))
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.ejectStone(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.driveForward(false, 2, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setElbowTargetPos(robot.crane.pos_AutoPark)) //extendBelt elbow to park
            .addState(() -> Math.abs(robot.crane.getElbowCurrentPos() - robot.crane.pos_AutoPark) < 20) //wait until done
            .build();

    public StateMachine craterSide_worlds_old = getStateMachine(autoStage)
            .addNestedStateMachine(autoSetupReverse)
            .addMineralState(mineralStateProvider, //turn to mineral
                    () -> robot.rotateIMU(39, TURN_TIME),
                    () -> true,
                    () -> robot.rotateIMU(321, TURN_TIME))

            //.addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.GOLD))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
            .addMineralState(mineralStateProvider,
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+800, 1, 10),
                    () -> robot.crane.extendToPosition(robot.crane.extendMid+1300, 1, 10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving,true))
            .addTimedState(.5f, () -> {}, () -> {})
            .addState(() -> robot.getArticulation() == PoseSkystone.Articulation.manual)
            .addSingleState(() -> robot.ledSystem.setColor(LEDSystem.Color.PURPLE))
            .addState(() -> robot.rotateIMU(85, 4)) //turn parallel to minerals
            .addState(() -> robot.driveForward(true, 1.5, DRIVE_POWER)) //drive to wall
            .addState(() -> robot.rotateIMU(135, 3)) //turn to depot
            //.addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.articulate(PoseSkystone.Articulation.manual, true))
            .addState(() -> robot.crane.setElbowTargetPos(10,1))
//            .addState(() -> robot.driveForward(true, .4, DRIVE_POWER))
            .addSingleState(() -> robot.crane.setBeltToElbowModeEnabled())
//            .addState(() -> robot.crane.extendToMax(1,10))
            .addSingleState(() -> robot.crane.setExtendABobTargetPos(robot.crane.extendMax))
            .addState(() -> robot.driveForward(true, .2, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetExtend())
            .addTimedState(DUCKY_TIME, //yeet ducky
                    () -> robot.crane.collect(),
                    () -> robot.crane.stopIntake())
            .addState(() -> robot.crane.extendToMid(1,10))
            .addSingleState(() -> robot.crane.setBeltToElbowModeDisabled())
            .addState(() -> robot.articulate(PoseSkystone.Articulation.reverseDriving, true))
            .addState(() -> robot.driveForward(false, .4, DRIVE_POWER))
            .addState(() -> robot.crane.nearTargetElbow())
            .addState(() -> robot.rotateIMU(34, 0.6))
            .addState(() -> robot.rotateIMU(315, 4))
            .addState(() -> robot.driveForward(true, 0.2, .8))
            .addState(() -> robot.crane.extendToMax())
            .build();

    public StateMachine driveStraight = getStateMachine(autoStage)
            .addState(() -> robot.driveForward(false, 4, DRIVE_POWER))
            .build();






    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> robot.resetMotors(true))
                .stateEndAction(() -> {})
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
