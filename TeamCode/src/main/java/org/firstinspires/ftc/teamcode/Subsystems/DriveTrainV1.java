package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;

import java.util.Locale;

public class DriveTrainV1 {
    private DcMotor fLeft, fRight, bLeft, bRight;
    //IMU variables
    private Gyroscope gyro;

    MovementDetectionClass detection;


    Telemetry telemetry;
    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1;
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    /**
     * Main constructor
     * @param telemetry telemetry
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     * @param gyro gyro
     */
    public DriveTrainV1(Telemetry telemetry, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight, Gyroscope gyro){
        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
        this.gyro=gyro;
        this.telemetry=telemetry;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving. "backward" to go backward
     * @param distance distance driving in inches
     */
    public void driveStraight(String direction, double distance) {
        resetEncoderWithoutEncoder();
        //sets direction of motors
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }
        double power=0.6*mDirection;

        //converts current position into inches
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        while(cPosition < distance){
            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            if(distance-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving. "backward" to go backward
     * @param distance distance driving in inches
     * @param power of drive wheels
     */
    public void driveStraight(String direction, double distance,double power) {
        resetEncoderWithoutEncoder();
        //sets direction of motors
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }
        power=power*mDirection;

        //converts current position into inches
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        while(cPosition < distance){
            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            if(distance-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


    /**
     * Resets drive encoders without running using encoders
     */
    public void resetEncoderWithoutEncoder(){
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * strafes for a given time
     * @param time time strafing(in milliseconds)
     * @param direction "left" for left "right" for right
     */
    public void strafeSeconds(double time, String direction){
        double stopTime=time+System.currentTimeMillis();
        //sets direction strafing
        int mDirection=1;
        if(direction.equals("left")){
            mDirection=-1;
        }

        while(System.currentTimeMillis()<stopTime){
            fLeft.setPower(0.45*mDirection);
            fRight.setPower(-0.45*mDirection);
            bLeft.setPower(-0.45*mDirection);
            bRight.setPower(0.45*mDirection);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }


    /**
     * turns to the desired angle
     * 0-360 in a clockwise format
     * @param destination
     */
    public void rotation(double destination) {
        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed = 0;
        double min = 0.18;
        double max = 0.8;
        double iTime=System.currentTimeMillis();

        //standard current angle
        double heading = gyro.getYaw();

        //check if over 360
        if (Math.abs(destination)>360) {
            //positive
            if(destination>0) {
                while(destination>360) {
                    destination-=360;
                    System.out.println(destination);
                }//end while
            }
            //negative
            else {
                while(destination<-360) {
                    destination+=360;
                    System.out.println(destination);
                }//end while
            }//end if else
        }//end greater than 360
        destination = Math.abs(destination);// convert to positive value
        if (destination==0) {//if 360 set to 0 as they are the same heading
            destination = 360;
        }
        if (heading==0) {//if 360 set to 0 as they are the same heading
            heading = 360;
        }

        //main phase of method
        while (heading < destination - 0.5 || heading > destination + 0.5) {
            telemetry.addData("heading",heading);
            telemetry.addData("speed",speed);
            telemetry.update();
            double delta = destination-heading; //the difference between destination and heading
            heading = gyro.getYaw();
            //decreases speed as robot approaches destination
            speed = (1 - ((heading) / destination)) * ((destination - heading) * 0.01);

            //if the speed gets under the min speed it will use the min speed
            if (Math.abs(speed) < min && Math.abs(speed) != 0) {
                speed = min;
            }
            //if the speed is over the max it will use max speed
            if(Math.abs(speed) > max){
                speed=max;
            }
            if (!(Math.abs(delta) == 360 || Math.abs(delta) == 0)) {//determine if we are at the intended heading
                if (((delta + 360) % 360) > 180) { //Chooses fastest route by determining if the arc length is longer to the right or left. Chooses fastest route by
                    fLeft.setPower(speed);
                    bLeft.setPower(speed);
                    bRight.setPower(-speed);
                    fRight.setPower(-speed);
                } else {
                    fLeft.setPower(-speed);
                    bLeft.setPower(-speed);
                    bRight.setPower(speed);
                    fRight.setPower(speed);
                }
            } else {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);
            }
            if(System.currentTimeMillis()>iTime+4500){//prevents method from going over 5 seconds
                break;
            }
        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    public void driveToLine(ColorSensor color, String lineColor, String dir){

        int theD;

        theD = dir.equals("forward") ? 1 : -1;

        while(color.red()<35&&color.blue()<35){
            fLeft.setPower(0.5*theD);
            bLeft.setPower(0.5*theD);
            bRight.setPower(0.5*theD);
            fRight.setPower(0.5*theD);
        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    public void strafeToLine(ColorSensor color, String lineColor, String dir){

        int theD;

        theD = dir.equals("left") ? 1 : -1;

        while(color.red()<35&&color.blue()<35){
            fLeft.setPower(-0.7*theD);
            bLeft.setPower(0.7*theD);
            bRight.setPower(-0.7*theD);
            fRight.setPower(0.7*theD);
        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    /**
     * turns to given angle without correction
     * @param distance angle destination
     * @param power drive power
     */
    public void simpleTurn(double distance,double power){
        telemetry.addData("angle",gyro.getRawYaw());
        if(distance<gyro.getRawYaw()){
            while(distance<gyro.getRawYaw()){
                telemetry.update();
                fLeft.setPower(power);
                bLeft.setPower(power);
                bRight.setPower(-power);
                fRight.setPower(-power);
            }
        }
        else{
            while(distance>gyro.getRawYaw()){
                telemetry.update();
                fLeft.setPower(-power);
                bLeft.setPower(-power);
                bRight.setPower(power);
                fRight.setPower(power);
            }
        }
        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    /**
     * strafe to a given range.
     * Need to call sensors distanceR and distanceL
     * @param dSensor distance sensor object
     * @param distance distance driving to
     * @param sLocal location of sensor(center,right,left)
     */
    public void driveRange(DistanceSensor dSensor, double distance, String sLocal){
        double cDistance=0;
        if(dSensor.getDistance(DistanceUnit.CM)>150){
            cDistance=150;
        }
        else{
            cDistance=dSensor.getDistance(DistanceUnit.CM);
        }
        int dir;
        if(sLocal.equals("center")) {
            if(distance<cDistance){
                while(distance<cDistance){
                    if(dSensor.getDistance(DistanceUnit.CM)>150){
                        cDistance=150;
                    }
                    else{
                        cDistance=dSensor.getDistance(DistanceUnit.CM);
                    }
                    telemetry.update();
                    fLeft.setPower(-0.4);
                    fRight.setPower(-0.4);
                    bLeft.setPower(-0.4);
                    bRight.setPower(-0.4);
                }
            }
            else{
                while(distance>cDistance) {
                    if(dSensor.getDistance(DistanceUnit.CM)>150){
                        cDistance=150;
                    }
                    else{
                        cDistance=dSensor.getDistance(DistanceUnit.CM);
                    }
                    fLeft.setPower(0.4);
                    fRight.setPower(0.4);
                    bLeft.setPower(0.4);
                    bRight.setPower(0.4);
                }
            }
        }
        else{
            if(sLocal.equals("right")){
                dir = -1;
            }
            else{
                dir = 1;
            }

            if(distance>dSensor.getDistance(DistanceUnit.CM)){
                while (distance>dSensor.getDistance(DistanceUnit.CM)) {
                    if(Math.abs(distance-dSensor.getDistance(DistanceUnit.CM))>20){
                        fLeft.setPower(0.7*dir);
                        fRight.setPower(-0.7*dir);
                        bLeft.setPower(-0.7*dir);
                        bRight.setPower(0.7*dir);
                    }
                    else{
                        fLeft.setPower(0.4*dir);
                        fRight.setPower(-0.4*dir);
                        bLeft.setPower(-0.4*dir);
                        bRight.setPower(0.4*dir);
                    }

                }
            }
            else{
                while (distance<dSensor.getDistance(DistanceUnit.CM)) {
                    if(Math.abs(distance-dSensor.getDistance(DistanceUnit.CM))>20){
                        fLeft.setPower(-0.7*dir);
                        fRight.setPower(0.7*dir);
                        bLeft.setPower(0.7*dir);
                        bRight.setPower(-0.7*dir);
                    }
                    else{
                        fLeft.setPower(-0.4*dir);
                        fRight.setPower(0.4*dir);
                        bLeft.setPower(0.4*dir);
                        bRight.setPower(-0.4*dir);
                    }
                }
            }
        }

        fLeft.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
        fRight.setPower(0);
    }

    /**
     * wait time. Can't be more than 5 seconds
     * @param wTime time delayed in milliseconds
     */
    public void delay(double wTime){
        double iTime=System.currentTimeMillis();
        while (System.currentTimeMillis()<iTime+wTime) {

        }
    }


}
