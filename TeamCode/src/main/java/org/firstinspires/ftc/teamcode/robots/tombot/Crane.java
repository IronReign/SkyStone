package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 2938061 on 11/10/2017.
 */

public class Crane {

    //number of ticks per revolution REV HD motor: 2240
    //number of ticks per revolution REV Core    : 288

    DcMotor elbow = null;
    DcMotor extendABob = null;
    DcMotor hook = null;

    Servo intakeRight = null;
    Servo intakeLeft = null;
    Servo intakeServoFront = null;
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

    int servoGateOpen;
    int servoGateClosed;

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

    //belt extension encoder values
    public  int extendDeposit;
    public  int extendMax;
    public  int extendMid;
    public  int extendLow; //clears hook and good for retracting prior to deposit without tipping robot
    public  int extendMin;  //prevent crunching collector tray
    public  int extendPreLatch = extendMax;

    public int stow = 650;

    public int currentTowerHeight;
    public final double blockHeightMeter = 0.127;
    public int anglePerBlock;

    public int craneArticulation = 0;


    private boolean hookUp = true;
    private int gripperState = 0;

    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 22.3296703;

    public boolean active = true;

    public Crane(DcMotor elbow, DcMotor extendABob, DcMotor hook, Servo intakeServoFront, Servo intakeServoBack, Servo gripperSwivel){

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(elbow.getCurrentPosition());
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //elbow.setDirection(DcMotorSimple.Direction.REVERSE);

        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendABob.setTargetPosition(extendABob.getCurrentPosition());
        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hook.setTargetPosition(extendABob.getCurrentPosition());
        hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hook.setPower(0);
        //hook.setDirection(DcMotorSimple.Direction.REVERSE);

        this.elbow = elbow;
        this.extendABob = extendABob;
        this.hook = hook;
        this.intakeServoFront = intakeServoFront;
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

        servoGateOpen = 1350;
        servoGateClosed = 800;

        motorHooked = 120;
        motorUnhooked = 5;
        motorMidHooked = 80;

        swivel_Right90 = 0;
        swivel_Front = 900;
        swivel_Left90 = 1556;
        swivel_left_Block = 800;
        swivel_Right_Block= 1000;

        //bel
        // t extension encoder values
        extendDeposit = 1489;
        extendMax = 2960;
        extendMid= 980;
        extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
        extendMin = 300;  //prevent crunching collector tray
    }
//
//    public Crane(DcMotor elbow, DcMotor extendABob, Servo hook, Servo intakeServoFront){
//
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elbow.setTargetPosition(elbow.getCurrentPosition());
//        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        elbow.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        extendABob.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        extendABob.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        extendABob.setTargetPosition(extendABob.getCurrentPosition());
//        extendABob.setDirection(DcMotorSimple.Direction.REVERSE);
//        //extendABobRight.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        //intakeGate.setDirection(Servo.Direction.REVERSE);
//
//        this.elbow = elbow;
//        this.extendABob = extendABob;
//        this.hook = hook;
//        this.intakeServoFront = intakeServoFront;
//        intakeServoBack.setDirection(Servo.Direction.REVERSE);
//
//        intakePwr = .3; //.35;
//        //normal Teleop encoder values
//        pos_preIntake = 3600;
//        pos_Intake   = 3900;
//        pos_Deposit  = 1520;
//        pos_reverseIntake = 1407;
//        pos_reversePreDeposit=1408;
//        pos_reverseDeposit = 3400;
//        pos_reverseSafeDrive = 1000;
//        pos_PartialDeposit = 1700;
//        pos_SafeDrive = 800;
//
//        //autonomous encoder values
//        pos_AutoPark = pos_SafeDrive + 500;
//        pos_autonPrelatch = 2950;
//
//        //end game encoder values
//        pos_prelatch = 2000;
//        pos_latched = 2764;
//        pos_postlatch = 1240;
//
//        servoGateOpen = 2200;
//        servoGateClosed = 900;
//
//        motorHooked = 1800;
//        motorUnhooked = 1300;
//
//        //belt extension encoder values
//        extendDeposit = 1489;
//        extendMax = 2960;
//        extendMid= 980;
//        extendLow = 650; //clears hook and good for retracting prior to deposit without tipping robot
//        extendMin = 300;  //prevent crunching collector tray
//    }


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
        updateIntake();
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

    public void updateIntake() {
        switch(intakeState) {
            case 0:
                //stopIntake();
                break;
            case 1:
                collect();
                break;
            case 2:
                ejectStone();
                break;
            case 3:
            default:
                //do nothing
                break;
        }
    }

    public void setIntakeModeOff() {
        intakeState = 0;
    }
    public void setIntakeModeIn() {
        intakeState = 1;
    }
    public void setIntakeModeOut() {
        intakeState = 2;
    }
    public void setIntakeModeManual() {
        intakeState = 3;
    }

    public void setTowerHeight(int newHeight){
        currentTowerHeight += newHeight;
    }

    public int getCurrentTowerHeight(){
        return currentTowerHeight;
    }

    int hypotenuse = 0;
    public void extendToTowerHeight(){
        hypotenuse = (int)(Math.sqrt(.25 + Math.pow(((currentTowerHeight+1)* blockHeightMeter),2)));//in meters
        setElbowTargetPos((int)(ticksPerDegree*Math.acos(.5/ hypotenuse)),1);
        setExtendABobTargetPos((int)(hypotenuse *(107.0/2960.0)));
    }


    public void hookOn(){

        hook.setTargetPosition(motorHooked);
        hook.setPower(hookPwr);
        hookUp = false;
    }
    public void hookOff(){
        hook.setTargetPosition(motorMidHooked);
        hook.setPower(hookPwr);
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
            gripperSwivel.setPosition(.7);
        else
            gripperSwivel.setPosition(.3);
    }

    public void stopSwivel(){
        gripperSwivel.setPosition(.5);
    }

    //This is for auto
    public boolean setGripperSwivelRotation(int encodedPosition){
        gripperSwivel.setPosition(servoNormalize(encodedPosition));
        return true;
    }

    public boolean grabStone(){
        intakeServoFront.setPosition(servoNormalize(servoGateClosed));
        //intakeServoBack.setPosition(servoNormalize(servoGateOpen));
        gripperState = 1;
        return true;
    }
    public boolean ejectStone(){
        intakeServoFront.setPosition(servoNormalize(servoGateOpen));
        //intakeServoBack.setPosition(servoNormalize(servoGateClosed));
        gripperState = 2;
        return true;
    }
    public void stopGripper() {
        intakeServoFront.setPosition(servoNormalize(1500));
        //intakeServoBack.setPosition(servoNormalize(1500));
        gripperState = 0;
    }

    public void stopIntake(){

    }


    boolean switcha = false;
    public void toggleGripper() {
        if(switcha == false) {
            grabStone();
            switcha=true;
        }
        else {
            ejectStone();
            switcha = false;
        }
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
        elbowPos = pos;
    }
    public boolean setElbowTargetPos(int pos, double speed){
        setElbowTargetPos(pos);
        setElbowPwr(speed);
        if (nearTargetElbow()) return true;
        else return false;
    }
    public int getElbowTargetPos(){
        return elbowPos;
    }
    public int getElbowCurrentPos(){
        return elbow.getCurrentPosition();
    }
    public double getCurrentAngle(){return  elbow.getCurrentPosition()/ticksPerDegree;}
    public double getCurrentLength(){
        return (107.0/2960.0)*getExtendABobCurrentPos() + 46;
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
        if ((Math.abs( getElbowCurrentPos()-getElbowTargetPos()))<15) return true;
        else return false;
    }
    public boolean nearTarget(){
        if (nearTargetElbow() && nearTargetExtend()) return true;
        else return false;
    }

    public void increaseElbowAngle(){
        setElbowTargetPos(Math.min(getElbowCurrentPos() + 100, pos_Intake));
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

    public void runToAngle(double angle){
        setElbowTargetPos((int)(angle * ticksPerDegree));
    }//untested

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
}

