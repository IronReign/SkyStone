
//written by Cooper Clem, 2019
package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Turret{
    //motor
    private DcMotor turnTable  = null;
    private double turntablePow = 1;
    private double safeTurn = .5;

    //Position variables
    private int currentDegrees;
    public int currentRotation;
    public  int currentRotationInternal;
    public int degreesSinceBegin;
    private double ticksPerDegree;
    private boolean active = true;

    //positions
    private int a90degreesleft;
    private int a90degreesright;
    private int a180degrees;
    private int a360degrees;

    public Turret(DcMotor turnTable) {

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setTargetPosition(turnTable.getCurrentPosition());
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.turnTable = turnTable;
        currentDegrees= 0;
        currentRotation= 0;
        currentRotationInternal= 0;
        degreesSinceBegin= 0;
        ticksPerDegree= 1;
        a90degreesleft= 90;
        a90degreesright= 90;
        a180degrees= 180;
        a360degrees= 360;
    }

    public void update(){
        if(active) { //don't keep updating if we are retractBelt to target position

            currentRotation = currentRotationInternal;
            turnTable.setTargetPosition(currentRotation);
            turnTable.setPower(turntablePow);
        }
    }

    public boolean isActive(){
        return active;
    }

    public void rotateRight(double power){
        currentRotationInternal += ticksPerDegree;
        degreesSinceBegin += ticksPerDegree;
        turntablePow = power* .1;
    }

    public void rotateLeft(double power){
        currentRotationInternal -= ticksPerDegree;
        degreesSinceBegin -= ticksPerDegree;
        turntablePow = power * .1;
    }

    public void setTurntablePosition(int position, double power) {
        currentRotationInternal = position;
        turntablePow = power;
    }

    public void setRotation90(boolean right) {
        if(right == true) {
            currentRotationInternal += currentRotationInternal%a90degreesright;
            turntablePow = safeTurn;
        }
        else {
            currentRotationInternal -= currentRotationInternal%a90degreesleft;
            turntablePow = safeTurn;
        }
    }

    public void setRotation180() {
        if(currentRotation <= 0)
            currentRotation -= currentRotationInternal%a180degrees;
        if(currentRotation >= 0)
            currentRotation += currentRotationInternal%a180degrees;
        if(currentRotation == 0)
            currentRotation -= a180degrees;
        turntablePow = safeTurn;
    }

    public void setToFront(){
        if(currentRotationInternal < a360degrees ||  currentRotationInternal > -a360degrees)
            currentRotationInternal = 0;
        else if(currentRotationInternal < 0) {
            currentRotationInternal += currentRotationInternal %a360degrees;
            turntablePow = safeTurn;
        }
        else {
            currentRotationInternal -= currentRotationInternal % a360degrees;
            turntablePow = safeTurn;
        }

    }

    public int getCurrentRotationInternal(){
        return turnTable.getCurrentPosition();
    }

    public void returnToZero() {
        currentRotationInternal = 0;
        turntablePow = safeTurn;
    }

    public void resetEncoder() {
        //just encoders - only safe to call if we know collector is in normal starting position
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
