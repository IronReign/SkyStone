package org.firstinspires.ftc.teamcode.util;

public class Conversions {

    public Conversions(){}

    public static double servoNormalize(int pulse){
        double normalized = (double)pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }

}
