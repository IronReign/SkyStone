
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

    //team members
    boolean BruhnaviyaIsOn = false;
    String Bruhnaviya = "canceled";

    public Turret(DcMotor turnTable) {

        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setTargetPosition(turnTable.getCurrentPosition());
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setDirection(DcMotorSimple.Direction.REVERSE);

        this.turnTable = turnTable;
        currentDegrees= 0;
        currentRotation= 0;
        ticksPerDegree = 170/90;
        currentRotationInternal= 0;
        degreesSinceBegin= 0;
        ticksPerDegree= 2;
        a90degreesleft= -170;
        a90degreesright= 170;
        a180degrees= 340;
        a360degrees= 680;
    }

    public void update(){
        if(active && currentRotationInternal != currentRotation) { //don't keep updating if we are retractBelt to target position
            currentRotation = currentRotationInternal;
            turnTable.setTargetPosition(currentRotationInternal);
            turnTable.setPower(turntablePow);
        }
        else
            turnTable.setPower(0);
    }

    public boolean isActive(){
        return active;
    }
    public void setActive(boolean active){this.active = active;}

    public void rotateRight(double power){
        setTurntablePosition(getCurrentRotation() + 5, power);
        degreesSinceBegin += 5;
    }

    public void rotateLeft(double power){
        setTurntablePosition(getCurrentRotation() - 5, power);
        degreesSinceBegin -= 5;
    }

    public void setTurntablePosition(int position, double power) {
        currentRotationInternal = position;
        turntablePow = power;
    }

    public void setRotation90(boolean right) {
        if(right == true) {
            currentRotation += turnTable.getCurrentPosition()%a90degreesright;
            turntablePow = safeTurn;
        }
        else {
            currentRotationInternal -= turnTable.getCurrentPosition()%a90degreesleft;
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
        if(getCurrentRotation() < a360degrees ||  currentRotationInternal > -a360degrees)
             setTurntablePosition(0,safeTurn);
        else if(getCurrentRotation() < 0) {
            setTurntablePosition(getCurrentRotation() + getCurrentRotation() %a360degrees, .5);
        }
        else {
            setTurntablePosition(getCurrentRotation() - getCurrentRotation() %a360degrees, .5);
        }

    }

    public int getCurrentRotationEncoderRaw(){
        return turnTable.getCurrentPosition();
    }
    public int getCurrentRotation(){
        return currentRotation;
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
