
//written by Cooper Clem, 2019
package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Turret{
    //motor
    private DcMotor turnTable  = null;
    private double turnTableSpeed = 1;
    private double safeTurn = .5;

    //Position variables
    public int targetRotationTicks;
    private double ticksPerDegree;
    private boolean active = true;

    //positions
    private int a90degrees;
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
        targetRotationTicks = 0;
        ticksPerDegree = 170/90;
        targetRotationTicks= 0;
        a90degrees= (int) (ticksPerDegree*90);
        a180degrees= (int) (ticksPerDegree*180);
        a360degrees= (int) (ticksPerDegree*360);
        setActive(true);
    }

    public void update(){
        if(active && targetRotationTicks != turnTable.getCurrentPosition()) { //don't keep updating if we are retractBelt to target position
            turnTable.setTargetPosition(targetRotationTicks);
            turnTable.setPower(turnTableSpeed);
        }
    }

    public boolean isActive(){
        return active;
    }
    public void setActive(boolean active){
        this.active = active;
        if(active = true)
            turnTable.setPower(.5);
        else
            turnTable.setPower(0);
    }

    public void rotateRight(double power){ setTurntablePosition(getCurrentRotationEncoderRaw() + 5, power);}

    public void rotateLeft(double power){setTurntablePosition(getCurrentRotationEncoderRaw() - 5, power);}

    public void setTurntablePosition(int position, double power) {
        targetRotationTicks = position;
        turnTableSpeed = power;
    }

    public void setRotation90(boolean right) {
        if(right == true) {
            targetRotationTicks += turnTable.getCurrentPosition()%a90degrees;
            turnTableSpeed = safeTurn;
        }
        else {
            targetRotationTicks -= turnTable.getCurrentPosition()%-a90degrees;
            turnTableSpeed = safeTurn;
        }
    }

    public void setRotation180() {
        if(getCurrentRotationEncoderRaw() < 0)
            targetRotationTicks -= getCurrentRotationEncoderRaw()%a180degrees;
        if(getCurrentRotationEncoderRaw() > 0)
            targetRotationTicks += getCurrentRotationEncoderRaw()%a180degrees;
        if(getCurrentRotationEncoderRaw() == 0)
            targetRotationTicks -= a180degrees;
        turnTableSpeed = safeTurn;
    }

    public void setToFront(){
        if(getTargetRotationTicks() < a360degrees ||  targetRotationTicks > -a360degrees)
            setTurntablePosition(0,safeTurn);
        else if(getTargetRotationTicks() < 0) {
            setTurntablePosition(getTargetRotationTicks() + getTargetRotationTicks() %a360degrees, .5);
        }
        else {
            setTurntablePosition(getTargetRotationTicks() - getTargetRotationTicks() %a360degrees, .5);
        }

    }

    public int getCurrentRotationEncoderRaw(){
        return turnTable.getCurrentPosition();
    }
    public int getTargetRotationTicks(){
        return targetRotationTicks;
    }

    public void returnToZero() {
        targetRotationTicks = 0;
        turnTableSpeed = safeTurn;
    }

    public void resetEncoder() {
        //just encoders - only safe to call if we know collector is in normal starting position
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
