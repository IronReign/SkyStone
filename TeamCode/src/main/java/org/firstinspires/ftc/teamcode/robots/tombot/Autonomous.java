package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.statemachine.MineralStateProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.vision.SkystonePos;
import org.firstinspires.ftc.teamcode.vision.Viewpoint;
import org.firstinspires.ftc.teamcode.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.vision.VisionProvidersSkystone;

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
    public VisionProvidersSkystone vps;
    public int visionProviderState;
    public boolean visionProviderFinalized;
    public boolean enableTelemetry = false;
    public static final Class<? extends VisionProvider>[] visionProviders = VisionProviders.visionProviders;
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
        //Turn on camera to see which is gold
//        SkystonePos pos = vps.detect();
        // Hold state lets us know that we haven't finished looping through detection
            telemetry.addData("Vision Detection", "GoldPos: %.2f", 0.0);
            //vps.shutdownVision();
            return true;
//        else {
//            telemetry.addData("Vision Detection", "HOLD_STATE (still looping through internally)");
//            return false;
//
    }

//    public StateMachine visionTest = getStateMachine(autoStage)
//            .addState(() -> {
//             robot.xPos = robot.vps.detect();
//             return false;
//            })
//            .build();



    public StateMachine redAutoFull = getStateMachine(autoStage)

            .addState(() -> robot.crane.toggleGripper())
            .addState(() -> robot.crane.setGripperSwivelRotation(1600))
//            .addState(() -> sample())
            .addState(() -> (robot.crane.setElbowTargetPos(300,.8)))
            .addMineralState(mineralStateProvider,
                    () -> { robot.turret.rotateIMUTurret(340,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_left_Block);},
                    () -> true,
                    () -> { robot.turret.rotateIMUTurret(20,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_Right_Block);})
            .addState(() ->robot.crane.extendToPosition(2160,.7,90))
            .addState(() ->robot.crane.setElbowTargetPos(40,.3))
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))
            //.addTimedState(6f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))// so we make sure everything is stopped
            //.addState(() -> robot.turret.setOffsetHeading(90.0))

            //.addTimedState(1f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))// so we make sure everything is stopped
            //.addState(() ->robot.rotateIMU(217, 9))//todo- make this a curve instead of following the hypotenuse
            .addTimedState(3f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))


            .addState(() -> (robot.driveIMUDistance(.6,90,true,1.91008)))//this and ^^^^ put the robot in front of the build plate
            .addState(() -> (robot.crane.setElbowTargetPos(100,.8)))
            .addState(() ->{robot.turret.rotateIMUTurret(90,6); return robot.rotateIMU(90,6);}) //gets the arm and the robot in their correct orientation for depositing
            .addState(() -> (robot.driveForward(true, .01, .30)))//this and ^^^^ put the robot in front of the build plate
            .addSingleState(() -> robot.crane.hookOn())
            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))// so we make sure everything is stopped
            .addSimultaneousStates(()->{robot.turret.rotateIMUTurret(270,20); return robot.driveForward(false,.1,1);})
            .addSingleState(() -> robot.crane.hookOff())
            .addState(() -> (robot.driveForward(true, 1, .30)))//this and ^^^^ put the robot in front of the build plate
            .build();



    public StateMachine redAutoFullSecondary = getStateMachine(autoStage)
//            .addTimedState(autoDelay, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
//            .addState(() -> sample())
            .addState(() -> (robot.crane.setElbowTargetPos(300,.8)))
            .addState(() -> robot.crane.toggleGripper())
            .addMineralState(mineralStateProvider,
                    () -> { robot.turret.rotateIMUTurret(340,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_left_Block);},
            () -> true,
            () -> { robot.turret.rotateIMUTurret(20,.4); return robot.crane.setGripperSwivelRotation(robot.crane.swivel_Right_Block);})
            .addState(() ->robot.crane.extendToPosition(2040,.7,90))
            .addState(() ->robot.crane.setElbowTargetPos(40,.1))
            .addState(() -> robot.crane.toggleGripper())
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))

            .addTimedState(2f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))// so we make sure everything is stopped
            .addState(() ->robot.rotateIMU(75, 8))//todo- make this a curve instead of following the hypotenuse
            .addState(() -> (robot.driveForward(true, 1.91008, .80)))//this and ^^^^ put the robot in front of the build plate
            .addState(() ->{robot.turret.rotateIMUTurret(0,3); return robot.rotateIMU(90,3);}) //gets the arm and the robot in their correct orientation for depositing
            .addTimedState(10f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.extendToTowerHeightArticulation))
            .addState(() -> robot.crane.toggleGripper())//deposit stone
            .addSingleState(() -> robot.crane.changeTowerHeight(1))
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //note for now this puts the arm at 90

            //todo - this is the first block
            //this is the stuff that can be copied for each stone sequentially, just adjust the values
            .addState(() -> (robot.driveForward(false, 1.651, .80)))//puts the robot in front of the block in the quarry
            .addState(() ->{robot.turret.rotateIMUTurret(0,3); return robot.crane.setElbowTargetPos(40,.1);})//sets the arm up to be extended over the block
            .addState(() ->robot.crane.extendToPosition(2200,.7,90)) //puts the arm over the bolck
            .addState(() ->robot.crane.setElbowTargetPos(40,.1)) //crumch block
            .addState(() -> robot.crane.toggleGripper()) // yum!
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE")) //note that this is necessary for the toggle gripper to work
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //see line 135
            .addState(() -> (robot.driveForward(true, 1.651, .80))) //goes back to the build platform
            .addState(() ->robot.turret.rotateIMUTurret(0,3)) //makes the turret face the build plate
            .addTimedState(10f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.extendToTowerHeightArticulation))
            .addState(() -> robot.crane.toggleGripper()) //de-crompvch
            .addSingleState(() -> robot.crane.changeTowerHeight(1)) //incements the tower height by 1 for the next block
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //restarts it in the original position of this block

            //todo- this is the second block
            .addState(() -> (robot.driveForward(false, 2.21996, .80)))//puts the robot in front of the block in the quarry
            .addState(() ->{robot.turret.rotateIMUTurret(0,3); return robot.crane.setElbowTargetPos(40,.1);})//sets the arm up to be extended over the block
            .addState(() ->robot.crane.extendToPosition(2200,.7,90)) //puts the arm over the bolck
            .addState(() ->robot.crane.setElbowTargetPos(40,.1)) //crumch block
            .addState(() -> robot.crane.toggleGripper()) // yum!
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE")) //note that this is necessary for the toggle gripper to work
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //see line 135
            .addState(() -> (robot.driveForward(true, 2.21996, .80))) //goes back to the build platform
            .addState(() ->robot.turret.rotateIMUTurret(0,3)) //makes the turret face the build plate
            .addTimedState(10f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.extendToTowerHeightArticulation))
            .addState(() -> robot.crane.toggleGripper()) //de-crompvch
            .addSingleState(() -> robot.crane.changeTowerHeight(1)) //incements the tower height by 1 for the next block
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //restarts it in the original position of this block

            //todo- this is the third block
            .addState(() -> (robot.driveForward(false, 2.39776, .80)))//puts the robot in front of the block in the quarry
            .addState(() ->{robot.turret.rotateIMUTurret(0,3); return robot.crane.setElbowTargetPos(40,.1);})//sets the arm up to be extended over the block
            .addState(() ->robot.crane.extendToPosition(2200,.7,90)) //puts the arm over the bolck
            .addState(() ->robot.crane.setElbowTargetPos(40,.1)) //crumch block
            .addState(() -> robot.crane.toggleGripper()) // yum!
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE")) //note that this is necessary for the toggle gripper to work
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //see line 135
            .addState(() -> (robot.driveForward(true, 2.39776, .80))) //goes back to the build platform
            .addState(() ->robot.turret.rotateIMUTurret(0,3)) //makes the turret face the build plate
            .addTimedState(10f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.extendToTowerHeightArticulation))
            .addState(() -> robot.crane.toggleGripper()) //de-crompvch
            .addSingleState(() -> robot.crane.changeTowerHeight(1)) //incements the tower height by 1 for the next block
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //restarts it in the original position of this block

            //todo- this is the 4th block
            .addState(() -> (robot.driveForward(false, 2.62636, .80)))//puts the robot in front of the block in the quarry
            .addState(() ->{robot.turret.rotateIMUTurret(0,3); return robot.crane.setElbowTargetPos(40,.1);})//sets the arm up to be extended over the block
            .addState(() ->robot.crane.extendToPosition(2200,.7,90)) //puts the arm over the bolck
            .addState(() ->robot.crane.setElbowTargetPos(40,.1)) //crumch block
            .addState(() -> robot.crane.toggleGripper()) // yum!
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE")) //note that this is necessary for the toggle gripper to work
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //see line 135
            .addState(() -> (robot.driveForward(true, 2.62636, .80))) //goes back to the build platform
            .addState(() ->robot.turret.rotateIMUTurret(0,3)) //makes the turret face the build plate
            .addTimedState(10f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.extendToTowerHeightArticulation))
            .addState(() -> robot.crane.toggleGripper()) //de-crompvch
            .addSingleState(() -> robot.crane.changeTowerHeight(1)) //incements the tower height by 1 for the next block
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower)) //restarts it in the original position of this block

            //the ending sequnce todo- test how this actually should funtion
            .addState(() ->{robot.turret.rotateIMUTurret(270,3); return robot.rotateIMU(0,3);}) // sets the robot facing the build plate and the arm in the most convinent position
            .addState(() -> (robot.driveForward(true, .1, .2))) //goes up to the build plate
            .addSingleState(() -> robot.crane.hookOn()) //idk what this does
            .addTimedState(.5f, () -> telemetry.addData("DELAY", "STARTED"), () -> telemetry.addData("DELAY", "DONE"))
            .addState(() -> (robot.driveForward(false, 1, .2)))//drives back to the edge of the field
            .addState(() ->robot.rotateIMU(90, 8))//yeets the build plate in the corner
            .addSingleState(() -> robot.crane.hookOff())//seriously what does this do? if only the name of the method gave me some sort of indication
            .addState(() ->robot.rotateIMU(90, 8)) //faces the back of the robot to the bridge
            .addSingleState(() -> robot.articulate(PoseSkystone.Articulation.retractFromTower))//puts it in position to go under the bridge
            .addState(() -> (robot.driveForward(false, 1, .2))) // drives back and parks
            .build();

    public StateMachine primaryRedOld = getStateMachine(autoStage)
            //pick up stone
            .addState(() -> (robot.driveForward(true, .106, .80)))
            .addState(() ->robot.crane.setElbowTargetPos(340,1.0))
            .addState(() ->{robot.crane.extendToPosition(2000,1,8); return robot.crane.ejectStone();})
            .addState(() ->robot.crane.setElbowTargetPos(40,1.0))
            .addTimedState(5, //yeet ducky
                    () -> robot.crane.grabStone(),
                    () -> robot.crane.grabStone())
            .addState(() ->robot.crane.extendToPosition(100,1,20))
            .addState(() ->robot.crane.setElbowTargetPos(340,1))

            .addState(() -> (robot.rotateIMU(90, 4))) //rotate back toward building zone - stay in 2nd column
            .addState(() -> (robot.driveForward(true, .14, .80))) //todo: should continue until upward sensor detect transit of sky bridge - for now continuing until we are past bridge under odometry
            .addState(() -> (robot.rotateIMU(0, 3))) //turn back to foundation
            //deposit stones
            .addState(() -> {robot.crane.extendToPosition(100,.8,5); return robot.crane.setElbowTargetPos(100,1.0);})
            .addState(() ->robot.crane.ejectStone())

//            //park
//            .addState(() -> (robot.rotateIMU(270, 3)))
//            .addState(() ->robot.crane.setElbowTargetPos(3200,1.0)) //todo: should be an articulation to bridgetransit - for now lowering arm to known position to go under bridge
//            .addState(() -> (robot.driveForward(true, 2.5, .80))) //return to center set of quarry stones
//            .addState(() -> (robot.rotateIMU(0, 3))) //turn toward stones - should be a turret operation
            .build();

    public StateMachine walkOfShameRed = getStateMachine(autoStage)
            .addState(() -> (robot.rotateIMU(270, 3)))
            .build();

    public StateMachine autoMethodTesterTool = getStateMachine(autoStage)
            .addState(() -> (robot.driveIMUDistance(.1,0,true,1)))
            .build();

    public StateMachine walkOfShameBlue = getStateMachine(autoStage)
            //.addState(() -> (robot.driveForward(true, 68.8, .80))) //forward to 2nd column of tiles
            .addState(() ->robot.crane.setElbowTargetPos(-10,1))
            .addState(() -> (robot.rotateIMU(90, 4)))
            .build();

    public StateMachine primaryBlueOld = getStateMachine(autoStage)
            .addState(() -> (robot.driveForward(true, .608, .80))) //forward to 2nd column of tiles
            .addState(() -> (robot.rotateIMU(90, 4))) // rotate toward audience
            .addState(() -> (robot.driveForward(true, .5, .80))) //forward to wall - todo: change to stop with distance sensor insteasd of odometry
            .addState(() -> (robot.rotateIMU(0, 4))) //rotate back toward facing quarry
            //pick up skystone
            .addState(() -> (robot.rotateIMU(270, 4))) //rotate back toward building zone - stay in 2nd column
            .addState(() ->robot.crane.setElbowTargetPos(3200,1.0)) //todo: should be an articulation to bridgetransit - for now lowering arm to known position to go under bridge
            .addState(() -> (robot.driveForward(true, 1.3, .80))) //todo: should continue until upward sensor detect transit of sky bridge - for now continuing until we are past bridge under odometry
            .addState(() ->robot.crane.setElbowTargetPos(1500,1.0)) //todo: should be an articulation to raise crane to current tower level - for now just raising a fair bit
            .addState(() -> (robot.driveForward(true, 1.3, .80))) //todo: should continue until proximity from back wall detected
            .addState(() -> (robot.rotateIMU(0, 3))) //turn back to foundation
            //deposit stones
            .addState(() -> (robot.rotateIMU(90, 3)))
            .addState(() ->robot.crane.setElbowTargetPos(3200,1.0)) //todo: should be an articulation to bridgetransit - for now lowering arm to known position to go under bridge
            .addState(() -> (robot.driveForward(true, 2.5, .80))) //return to center set of quarry stones
            .addState(() -> (robot.rotateIMU(0, 3))) //turn toward stones - should be a turret operation
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
            vp = VisionProviders.defaultProvider.newInstance();
            //vp.initializeVision(robot.hwMap, telemetry, enableTelemetry, viewpoint);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException(e);
        }
        visionProviderFinalized = true;
    }

}
