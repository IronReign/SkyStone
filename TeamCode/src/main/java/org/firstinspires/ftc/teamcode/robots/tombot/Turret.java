
//written by Cooper Clem, 2019
package org.firstinspires.ftc.teamcode.robots.tombot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngleMinus;

public class Turret{
    //motor
    private  DcMotor motor = null;
    private double motorPwr = 1;
    private double safeTurn = .5;
    long turnTimer;
    boolean turnTimerInit;
    private double minTurnError = 1.0;

    //Position variables
    private int targetRotationTicks;
    private double ticksPerDegree;
    private boolean active = true;

    //positions
    private int a90degrees;
    private int a180degrees;
    private int a360degrees;

    //PID
    PIDController turretPID;
    private double kpTurret = 0.04; //proportional constant multiplier
    private double kiTurret = 0.0; //integral constant multiplier
    private double kdTurret= 0.0; //derivative constant multiplier
    double correction = 0.00; //correction to apply to turret motor

    //IMU
    BNO055IMU turretIMU;
    double turretRoll;
    double turretPitch;
    double turretHeading = 0;
    boolean initialized = false;
    private  double offsetHeading;
    private double offsetPitch;
    private double offsetRoll;
    private double turretTargetHeading = 0.0;
    Orientation imuAngles;
    boolean maintainHeadingInit;

    public Turret(DcMotor motor, BNO055IMU turretIMU) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motor = motor;
        targetRotationTicks = 0;
        ticksPerDegree = 170/90;
        targetRotationTicks= 0;
        a90degrees= (int) (ticksPerDegree*90);
        a180degrees= (int) (ticksPerDegree*180);
        a360degrees= (int) (ticksPerDegree*360);
        setActive(true);

        turretTargetHeading=0;
        turretPID = new PIDController(0,0,0);
        initIMU(turretIMU);

    }

    public void initIMU(BNO055IMU turretIMU){

        //setup Turret IMU
        BNO055IMU.Parameters parametersIMUTurret = new BNO055IMU.Parameters();
        parametersIMUTurret.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersIMUTurret.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMUTurret.loggingEnabled = true;
        parametersIMUTurret.loggingTag = "turretIMU";


        turretIMU.initialize(parametersIMUTurret);
        this.turretIMU=turretIMU;

    }

    public void update(boolean isActive){
        //IMU Update
        imuAngles= turretIMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        if (!initialized) {
            //first time in - we assume that the robot has not started moving and that orientation values are set to the current absolute orientation
            //so first set of imu readings are effectively offsets

            offsetHeading = wrapAngleMinus(360 - imuAngles.firstAngle, turretHeading);
            offsetRoll = wrapAngleMinus(imuAngles.secondAngle, turretRoll);
            offsetPitch = wrapAngleMinus(imuAngles.thirdAngle, turretPitch);
            initialized = true;
        }
        turretHeading = wrapAngle((360-imuAngles.firstAngle), offsetHeading);


//            offsetHeading = wrapAngleMinus(360-imuAngles.firstAngle, turretHeading);
        if(isActive) {
            motor.setTargetPosition(targetRotationTicks);
            motor.setPower(motorPwr);
        }



        //experiment code
        maintainHeadingTurret(true);
    }

    public boolean isActive(){
        return active;
    }
    public void setActive(boolean active){
        this.active = active;
        if(active == true)
            if(motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                motor.setPower(.5);
        else
            motor.setPower(0);
    }

    public void rotateRight(double multiplier){
        if(getTurretTargetHeading() <359.9)
        setTurntableAngle(getHeading()+(1.0*multiplier));
    else
        turretTargetHeading = 0.1;
        //turretTargetHeading = 360- getTurretTargetHeading();
    }

    public void rotateLeft(double multiplier){
        if(getTurretTargetHeading() > .1)
            setTurntableAngle(getHeading()-(1.0*multiplier));
        else
            turretTargetHeading = 359.9;
            //turretTargetHeading = getTurretTargetHeading() - 360;
    }



    public void setTurntablePosition(int position, double power) {
        setTurretMotorMode(false);
        targetRotationTicks = position;
        motorPwr = power;
    }



    public void rotateCardinal(boolean right){
        int pos = (int) (turretTargetHeading/90.0);
        if(right)
            setTurntableAngle((pos+1)*90 % 360);
        else
            setTurntableAngle(pos-1*90 % 360);
    }

    //experiment method
    public void setTurntableAngle(double angle){
        turretTargetHeading=angle;
    }

    public void setPower(double pwr){
        motorPwr =pwr;
        //motor.setPower(pwr);
    }

    public boolean setRotation90(boolean right) {
        if(right == true) {
            targetRotationTicks += motor.getCurrentPosition()%a90degrees;
            motorPwr = safeTurn;
            return true;
        }
        else {
            targetRotationTicks -= motor.getCurrentPosition()%-a90degrees;
            motorPwr = safeTurn;
            return true;
        }
    }

//    public void setRotation180() {
//        if(getCurrentRotationEncoderRaw() < 0)
//            targetRotationTicks -= getCurrentRotationEncoderRaw()%a180degrees;
//        if(getCurrentRotationEncoderRaw() > 0)
//            targetRotationTicks += getCurrentRotationEncoderRaw()%a180degrees;
//        if(getCurrentRotationEncoderRaw() == 0)
//            targetRotationTicks -= a180degrees;
//        motorPwr = safeTurn;
//    }
//
//    public void setToFront(){
//        if(getTargetRotationTicks() < a360degrees ||  targetRotationTicks > -a360degrees)
//            setTurntablePosition(0,safeTurn);
//        else if(getTargetRotationTicks() < 0) {
//            setTurntablePosition(getTargetRotationTicks() + getTargetRotationTicks() %a360degrees, .5);
//        }
//        else {
//            setTurntablePosition(getTargetRotationTicks() - getTargetRotationTicks() %a360degrees, .5);
//        }

//    }

    public int getCurrentRotationEncoderRaw(){
        return motor.getCurrentPosition();
    }
    public int getTargetRotationTicks(){
        return targetRotationTicks;
    }

    public void returnToZero() {
        targetRotationTicks = 0;
        motorPwr = safeTurn;
    }

    public void resetEncoder() {
        //just encoders - only safe to call if we know collector is in normal starting position
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void movePIDTurret(double Kp, double Ki, double Kd, double currentAngle, double targetAngle) {
        //if (pwr>0) PID.setOutputRange(pwr-(1-pwr),1-pwr);
        //else PID.setOutputRange(pwr - (-1 - pwr),-1-pwr);

        //initialization of the PID calculator's output range, target value and multipliers
        turretPID.setOutputRange(-1, 1);
        turretPID.setPID(Kp, Ki, Kd);
        turretPID.setSetpoint(targetAngle);
        turretPID.enable();

        //initialization of the PID calculator's input range and current value
        turretPID.setInputRange(0, 360);
        turretPID.setContinuous();
        turretPID.setInput(currentAngle);

        //calculates the angular correction to apply
        correction = turretPID.performPID();

        //performs the turn with the correction applied
        setPower(correction);
    }

    public void setTurretMotorMode(boolean IMUMODE){
        if(IMUMODE) {motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
        else{motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    }

    /**
     * Rotate to a specific heading with a time cutoff in case the robot gets stuck and cant complete the turn otherwise
     * @param targetAngle the heading the robot will attempt to turn to
     * @param maxTime the maximum amount of time allowed to pass before the sequence ends
     */
    public boolean rotateIMUTurret(double targetAngle, double maxTime){
        setTurretMotorMode(true);
        turretTargetHeading = turretHeading;
        if(!turnTimerInit){ //intiate the timer that the robot will use to cut of the sequence if it takes too long; only happens on the first cycle
            turnTimer = System.nanoTime() + (long)(maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        movePIDTurret(kpTurret, kiTurret, kdTurret, turretHeading, targetAngle);
        //check to see if the robot turns within a threshold of the target
        if(Math.abs(turretHeading - targetAngle) < minTurnError) {
            turnTimerInit = false;
            setPower(0);
            return true;
        }
        if(turnTimer < System.nanoTime()){ //check to see if the robot takes too long to turn within a threshold of the target (e.g. it gets stuck)
            turnTimerInit = false;
            setPower(0);
            return true;
        }
        return false;
    }

    public void maintainHeadingTurret(boolean buttonState){

        //if the button is currently down, maintain the set heading
        if(buttonState) {
            //if this is the first time the button has been down, then save the heading that the robot will hold at and set a variable to tell that the heading has been saved
            if (!maintainHeadingInit) {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
               //turretTargetHeading = turretHeading;
                maintainHeadingInit = true;
            }
            //hold the saved heading with PID
            movePIDTurret(kpTurret,kiTurret,kdTurret,turretHeading,turretTargetHeading);
        }


        //if the button is not down, set to make sure the correct heading will be saved on the next button press
        if(!buttonState){
            maintainHeadingInit = false;
            setPower(0);
        }
    }

    public double getHeading(){
        return turretHeading;
    }

    public double getTurretTargetHeading(){
        return turretTargetHeading;
    }
    public double getCorrection(){return correction;}
    public double getMotorPwr(){return motorPwr;}
    public double getMotorPwrActual(){return motor.getPower();}

}
