package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.util.Conversions.futureTime;
import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Crane {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbow = null;
    DcMotor extendABob = null;
    Servo hook = null;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo servoGripper = null;
    Servo intakeServoBack = null;
    Servo gripperSwivel = null;

    double hookPwr = 1;

    int elbowPosInternal = 0;
    int elbowPos = 0;
    double elbowPwr = 1;

    int extendABobPosInternal = 0;
    int extendABobPos = 0;
    double extendABobPwr = 1;

    int intakeState = 3;
    boolean beltToElbowEnabled;

    public int motorHooked;
    public int motorUnhooked;
    public int motorMidHooked;

    int servoGripperOpen;
    int servoGripperClosed;

    public double intakePwr;
    //normal Teleop encoder values
    public int pos_preIntake;
    public int pos_Intake ;
    public int pos_Deposit;
    public int pos_reverseIntake;
    public int pos_reversePreDeposit;
    public int pos_reverseDeposit;
    public int pos_reverseSafeDrive;
    public int pos_PartialDeposit;
    public int pos_SafeDrive;
    public int swivel_Right90;
    public int swivel_Front;
    public int swivel_Left90;
    public int swivel_left_Block;
    public int swivel_Right_Block;
    private boolean gripperState;
    private int gripperSwivelState = 0;
    double hypotenuse = 0;

    //autonomous encoder values
    public int pos_AutoPark;
    public int pos_autonPrelatch;

    //end game encoder values
    public int pos_prelatch;
    public int pos_latched;
    public int pos_postlatch;
    public int glide = 80;
    public int autodepotthingy=350;

    //.374

    //elbow safety limits
    public int elbowMin = -50;
    public int elbowStart = 180; //put arm just under 18" from ground
    public int elbowMax = 1340; //measure this by turning on the robot with the elbow fully opened and then physically push it down to the fully closed position and read the encoder value, dropping the minus sign

    //belt extension encoder values
    public  int extendDeposit;
    public  int extendMax;
    public  int extendMid;
    public  int extendLow; //clears hook and good for retracting prior to deposit without tipping robot
    public  int extendMin;  //prevent crunching collector tray
    public  int extendPreLatch = extendMax;

    //foundation hook servo values
    public int foundation_hook_up = 1665;
    public int foundation_hook_down = 1163;

    public int stow = 650;

    public int currentTowerHeight;
    public final double blockHeightMeter = 0.127;
    public int anglePerBlock;

    public int craneArticulation = 0;


    private boolean hookUp = true;
    //private int gripperState = 0;

    public double ticksPerDegree = 19.4705882353;
    public final double ticksPerMeter = 806/.2921;

    public boolean active = true;

    public boolean getGripperState() {
        return gripperState;
    }

    public Crane(DcMotor elbow, DcMotor extendABob, Servo hook, Servo servoGripper, Servo intakeServoBack, Servo gripperSwivel){

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(elbow.getCurrentPosition());
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setTargetPosition(extendABob.getCurrentPosition());
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //hook.setDirection(DcMotorSimple.Direction.REVERSE);

        this.elbow = elbow;
        this.extendABob = extendABob;
        this.hook = hook;
        this.servoGripper = servoGripper;
        this.intakeServoBack = intakeServoBack;
        this.gripperSwivel = gripperSwivel;
        intakeServoBack.setDirection(Servo.Direction.REVERSE);

        intakePwr = .3; //.35;
        //normal Teleop encoder values
        pos_preIntake = 3600;
        pos_Intake   = 2660;
        pos_Deposit  = 1520;
        pos_reverseIntake = 1407;
        pos_reversePreDeposit=1408;
        pos_reverseDeposit = 3400;
        pos_reverseSafeDrive = 1000;
        pos_PartialDeposit = 1700;
        pos_SafeDrive = 800;
        currentTowerHeight = 0;

        //autonomous encoder values
        pos_AutoPark = pos_SafeDrive + 500;
        pos_autonPrelatch = 2950;

        //end game encoder values
        pos_prelatch = 2000;
        pos_latched = 2764;
        pos_postlatch = 1240;

        servoGripperOpen = 1350;
        servoGripperClosed = 800;

        motorHooked = 120;
        motorUnhooked = 5;
        motorMidHooked = 80;

        swivel_Right90 = 0;
        swivel_Front = 900;
        swivel_Left90 = 1556;
        swivel_left_Block = 800;
        swivel_Right_Block= 1000;

        //belt extension encoder values
        extendDeposit = 1489;
        extendMax = 2960;
        extendMid= 980;
        extendLow = 600; //clears foundation grabber at all times
        extendMin = 200;  //prevent crunching foundation grabber
        gripperState = false;
    }


    public void update(){
        if(active && elbowPosInternal!=elbowPos) { //don't keep updating if we are retractBelt to target position
            elbowPosInternal = elbowPos;
            elbow.setTargetPosition(elbowPos);
            elbow.setPower(elbowPwr);
        }
        if(active && extendABobPosInternal!=extendABobPos) { //don't keep updating if we are retractBelt to target position
            extendABobPosInternal = extendABobPos;
            extendABob.setTargetPosition(extendABobPos);
            extendABob.setPower(extendABobPwr);
        }
        updateGripper();
        updateBeltToElbow();
    }

    public void updateBeltToElbow() {
        if(beltToElbowEnabled) {
            setElbowTargetPos(beltToElbow(getExtendABobCurrentPos(), 0)-40);
        }
    }

    public void setBeltToElbowModeEnabled() {
        beltToElbowEnabled = true;
    }

    public void setBeltToElbowModeDisabled() {
        beltToElbowEnabled = false;
    }

    public int elbowToBelt(int elbow, int offset){
        return (int)(4.5*(elbow+ offset)) +620;
    }

    public int beltToElbow(int belt, int offset){
        return (int)(2.0/9 * ((belt+offset)-620)) ;
    }

    int grabState = 2;
    double grabTimer;

    public void updateGripper() {
        switch(grabState){
            case 0:
                servoGripper.setPosition(servoNormalize(2200));
                //if(setElbowTargetPos(elbow.getCurrentPosition(),.2)) {
                    grabTimer = futureTime(1);
                    grabState++;
                //}
                break;

            case 1:
                if (System.nanoTime() >= grabTimer) {
                    servoGripper.setPosition(servoNormalize(1700));
                    grabState++;
                }
                break;

        }

    }

    public void changeTowerHeight(int newHeightAddition){
        if(currentTowerHeight > 0 || newHeightAddition > 0)
        currentTowerHeight += newHeightAddition;
    }

    public int getCurrentTowerHeight(){
        return currentTowerHeight;
    }

    public int getElbowMax() {return elbowMax;}

    public void extendToTowerHeight(){
        hypotenuse = Math.sqrt(.76790169 + Math.pow(((currentTowerHeight+1)* blockHeightMeter),2));//in meters
        setElbowTargetAngle(Math.toDegrees(Math.acos(0.8763/ hypotenuse)));
        setExtendABobLengthMeters(hypotenuse-.3683);
    }

    public void extendToTowerHeight(int height){
        hypotenuse = Math.sqrt(.76790169 + Math.pow(((height+1)* blockHeightMeter),2));//in meters
        setElbowTargetAngle(Math.toDegrees(Math.acos(0.8763/ hypotenuse)));
        setExtendABobLengthMeters(hypotenuse-.3683);
    }

    public void hookOn(){

        hook.setPosition(servoNormalize(foundation_hook_down));
        hookUp = false;
    }
    public void hookOff(){
        hook.setPosition(servoNormalize(foundation_hook_up));
        hookUp = true;
    }

    public void hookToggle(){
        if(hookUp)
            hookOn();
        else
            hookOff();
    }

    public void swivelGripper(boolean right){
        if(right == true)
            gripperSwivel.setPosition(gripperSwivel.getPosition()-.02);
        else
            gripperSwivel.setPosition(gripperSwivel.getPosition()+.02);
    }

    public void toggleSwivel(){
        if(gripperSwivelState == 0) {
            gripperSwivel.setPosition(.5);
            gripperSwivelState++;
        }
        else if(gripperSwivelState == 1) {
            gripperSwivel.setPosition(1);
            gripperSwivelState++;
        }
        else if(gripperSwivelState == 2) {
            gripperSwivel.setPosition(.5);
            gripperSwivelState++;
        }
        else{
            gripperSwivel.setPosition(0);
            gripperSwivelState = 0;
        }
    }

    //This is for auto
    public boolean setGripperSwivelRotation(int encodedPosition){
        gripperSwivel.setPosition(servoNormalize(encodedPosition));
        return true;
    }

    public boolean grabStone(){
        servoGripper.setPosition(servoNormalize(servoGripperClosed));

        //gripperState = 1;
        return true;
    }
    public boolean ejectStone(){
        servoGripper.setPosition(servoNormalize(servoGripperOpen));
        //gripperState = 2;
        return true;
    }
    public boolean setGripperPos(boolean open){
        if(open)
            servoGripper.setPosition(servoNormalize(servoGripperOpen));
        else
            servoGripper.setPosition(servoNormalize(servoGripperClosed));
        return true;
    }
    public void stopGripper() {
        servoGripper.setPosition(servoNormalize(1500));

        //gripperState = 0;
    }

    public void stopIntake(){

    }

    //this is a behavior to calibrate the crane including the elbow, arm and gripper
    //keep calling until it returns true
    //arm should be pointed mostly up and mostly retracted
    //first raise the elbow on low power so it stalls on physical limit of hitting the extension motor shaft
    //must be run without encoders so power won't ramp up
    //retract the arm (extendabob) until it stops on low power - this will be its zero position

    int calibrateStage=0;
    double calibrateTimer;

    public boolean calibrate(){

        switch(calibrateStage){
            case 0:
                setMotorsForCalibration();
                elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elbow.setPower(.15);
                extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                extendABob.setPower(-.20); //retract to zero position
                calibrateTimer = futureTime(4.0f); //allow enough time for elbow to open fully and arm to retract
                calibrateStage++;
                break;
            case 1:
                if (System.nanoTime() >= calibrateTimer){
                    extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this should be correct zero for extension
                    extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extendToLow(); //push out to the minimum extension - this will need to be pulled back to zero for starting position

                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //temporarily zero at top of travel
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //elbow.setTargetPosition(-elbowMax); //normally we set the target through a method, but we have to override the safety here
                    //extendToPosition(100,.2,15);
                    toggleSwivel();
                    toggleSwivel();
                    elbowPos=-elbowMax; //set target explicitly
                    elbow.setPower(.3); //set speed explicitly
                    calibrateTimer = futureTime(1.5f); //allow enough time for elbow to close fully
                    calibrateStage++;

                }
                break;
            case 2:
                if (System.nanoTime() >= calibrateTimer) {
                    elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //zero elbow at bottom of travel
                    elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elbow.setTargetPosition(elbowMin); //this should not generate a movement
                    calibrateTimer = futureTime(1f); //allow enough time to raise to starting position
                    calibrateStage++;
                }
                break;
            case 3: //reset elbow and arm to starting position
                if (System.nanoTime() >= calibrateTimer) {
                    elbow.setTargetPosition(elbowStart);
                    extendToPosition(0,.6,15);
                    calibrateTimer = futureTime(4.0f); //enough time for next stage - retract egain
                    calibrateStage++;

                }
                break;
            case 4: //one more retract of extension
                if (System.nanoTime() >= calibrateTimer) {
                    extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    extendABob.setPower(-.20); //retract to zero position
                    calibrateTimer = futureTime(1.0f); //allow enough time for next stage
                    calibrateStage++;
                }
                break;

            case 5: //final zero of extension
                if (System.nanoTime() >= calibrateTimer) {
                    extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // this should be correct zero for extension
                    extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    calibrateStage = 0;
                    return true;
                }
                break;


        }


        return false;
    }

    public void setMotorsForCalibration(){

            elbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendABob.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean toggleGripper() {
        grabState = 0;
        return true;
    }


    public void collect(){
        intakeLeft.setPosition(.5 + intakePwr);
        intakeRight.setPosition(.5 + intakePwr);
    }
//    public void eject(){
//        intakeRight.setPosition(.5 - intakePwr);
//        intakeLeft.setPosition(.5 - intakePwr);}
//    public void stopIntake(){
//        intakeRight.setPosition(.5);
//        intakeLeft.setPosition(.5);
//    }


    public void setExtendABobTargetPos(int pos){
        extendABobPos = pos;
    }
    public int getExtendABobTargetPos(){
        return extendABobPos;
    }
    public int getExtendABobCurrentPos(){
        return extendABob.getCurrentPosition();
    }
    public void setExtendABobPwr(double pwr){ extendABobPwr = pwr; }

    public void setElbowTargetPos(int pos){
        if (pos>=elbowMin && pos<=elbowMax)
        elbowPos = pos;
    }
    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }

    public boolean setElbowTargetPosWithSlop(int pos, int slop, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbowWithSlop(slop)) return true;
        else return false;
    }

    public boolean setElbowTargetAngle(double angleDegrees){
        elbowPos =(int) (ticksPerDegree* angleDegrees);
        return true;
    }
    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbow.getCurrentPosition();
    }
    public double getCurrentAngle(){return  elbow.getCurrentPosition()/ticksPerDegree;}

    public void setExtendABobLengthMeters(double lengthMeters){
        setExtendABobTargetPos((int)(lengthMeters*ticksPerMeter));
    }

    public double getCurrentLengthInMeters(){
        return (ticksPerMeter)*getExtendABobCurrentPos();
    }
    public void setElbowPwr(double pwr){ elbowPwr = pwr; }

    public void stopAll(){
        setElbowPwr(0);
        setExtendABobPwr(0);
        update();
        active = false;
    }
    public void restart(double elbowPwr, double extendABobPwr){
        setElbowPwr(elbowPwr);
        setExtendABobPwr(extendABobPwr);
        active = true;
    }

    public void resetEncoders() {
        //just encoders - only safe to call if we know collector is in normal starting position
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public boolean extendToMin(){
        return extendToMin(extendABobPwr, 15);
    }
    public boolean extendToMin(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMin);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToLow(){
        return extendToLow(extendABobPwr, 15);
    }
    public boolean extendToLow(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendLow);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMid(){
        return extendToMid(extendABobPwr, 15);
    }
    public boolean extendToMid(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMid);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToPosition(int position, double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(position);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }
    public boolean extendToMax(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToMax(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendMax);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean extendToReverseDeposit(){
        return extendToMax(extendABobPwr, 15);
    }
    public boolean extendToReverseDeposit(double speed, int range){
        setExtendABobPwr(speed);
        setExtendABobTargetPos(extendDeposit);
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<range){
            return true;
        }
        return false;
    }

    public boolean nearTargetExtend(){
        if((Math.abs(getExtendABobCurrentPos()-getExtendABobTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTargetElbow(){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<55) return true;
        else return false;
    }
    public boolean nearTargetElbowWithSlop(int Slop){
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<Slop) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void increaseElbowAngle(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
    }

    public void adjustElbowAngle(double speed){
        if(extendABob.getCurrentPosition() > 600 && speed < 0 && elbow.getCurrentPosition() > 290)
            speed *= .8;
        if(extendABob.getCurrentPosition() > 1000 && speed < 0 && elbow.getCurrentPosition() > 290)
            speed *= .5;
        setElbowTargetPos(Math.max(getElbowCurrentPos() + (int)(200 * speed), elbowMin));


    }
    public void decreaseElbowAngle(){
        setElbowTargetPos(Math.max(getElbowCurrentPos() - 100, 0));
    }

    public void extendBelt(){
        setExtendABobTargetPos(Math.min(getExtendABobCurrentPos() + 100, extendMax));
    }

    public void retractBelt(){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() - 100, extendMin));
    }

    public void adjustBelt(double speed){
        setExtendABobTargetPos(Math.max(getExtendABobCurrentPos() + (int)(250 * speed), extendMin));
    }

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested


}

