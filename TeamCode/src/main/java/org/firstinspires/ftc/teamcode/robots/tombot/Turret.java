
//written by Cooper Clem, 2019

package org.firstinspires.ftc.teamcode.robots.tombot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PIDController;

import static org.firstinspires.ftc.teamcode.util.Conversions.nextCardinal;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrap360;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.Conversions.wrapAngleMinus;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
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
    public static double kpTurret = 0.02; //proportional constant multiplier goodish
    public static  double kiTurret = 0.01; //integral constant multiplier
    public static  double kdTurret= .05; //derivative constant multiplier
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
    private final double angleIncrement = 10;
    private boolean isMaintainingHeading = true;

    //sensors
    //DigitalChannel magSensor;


    public Turret(DcMotor motor, BNO055IMU turretIMU) {

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(motor.getCurrentPosition());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //this.magSensor = magSensor;

        this.motor = motor;
        ticksPerDegree = 170/90;
        targetRotationTicks= 0;
        a90degrees= (int) (ticksPerDegree*90);
        a180degrees= (int) (ticksPerDegree*180);
        a360degrees= (int) (ticksPerDegree*360);
        //setActive(true);

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

        //execute PID calcs
        if(isMaintainingHeading)
            maintainHeadingTurret(true);

    }

    /**
     * assign the current heading of the robot to a specific angle
     * @param angle the value that the current heading will be assigned to
     */
    public void setHeading(double angle){
        turretHeading = angle;
        initialized = false; //triggers recalc of heading offset at next IMU update cycle
    }

    //public boolean getMagSensorVal() {return magSensor.getState(); }

    public boolean isActive(){
        return active;
    }
    public void setActive(boolean active){
        this.active = active;
        if(active == true)
            if(motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) motor.setPower(.5);
        else
            motor.setPower(0);
    }
    public void adjust(double speed) {
        setTurntableAngle(getHeading(), 7.0 * speed);
    }

    public void rotateRight(double speed){

        setTurntableAngle(getHeading(), angleIncrement * speed);

    }

    public void toggleMaintainHeading(boolean isMaintainingHeading){this.isMaintainingHeading =! isMaintainingHeading;}

    public void rotateLeft(double speed){

        setTurntableAngle(getHeading(), angleIncrement * -speed);

    }

    public void setTurntablePosition(int position, double power) {
        setTurretMotorMode(false);
        targetRotationTicks = position;
        motorPwr = power;
    }

    public void rotateCardinalTurret(boolean right){

        setTurntableAngle(nextCardinal(getHeading(),right,10));
    }

    public void stopAll() {
        setPower(0);
        active = false;
    }

    public void setTurntableAngle(double currentAngle, double adjustAngle){
        turretTargetHeading=wrap360(currentAngle, adjustAngle);
    }

    public void setTurntableAngle(double angle){
        turretTargetHeading=wrap360(angle);
    }

    public void setPower(double pwr){
        motorPwr = pwr;
        motor.setPower(pwr);
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

//    public boolean calibrate(){
//        if ()
//    }

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
        if (isMaintainingHeading) setTurntableAngle(targetAngle);
        else movePIDTurret(kpTurret, kiTurret, kdTurret, turretHeading, targetAngle);

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

    public boolean setOffsetHeading(double new0){
        offsetHeading = new0;
        return true;
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
