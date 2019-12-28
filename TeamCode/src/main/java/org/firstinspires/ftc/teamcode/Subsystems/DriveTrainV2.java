package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;

public class DriveTrainV2 {
    private DcMotor fLeft, fRight, bLeft, bRight;
    //IMU variables
    private Gyroscope gyro;

    MovementDetectionClass detection;


    Telemetry telemetry;
    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1.5;    //From sprockets
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    double stopTime, stopDist;

    /**
     * Main constructor
     * @param telemetry telemetry
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     * @param gyro gyro
     */
    public DriveTrainV2(Telemetry telemetry, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight, Gyroscope gyro){
        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
        this.gyro=gyro;
        this.telemetry=telemetry;
    }

    public void setDist(double dist) {
        resetEncoderWithoutEncoder();
        stopDist = dist;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     */
    public boolean driveStraight(Directions direction) {
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.FORWARD) {
            mDirection = -1;
        }
        double power=0.6*mDirection;

        //converts current position into inches
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        if(cPosition < stopDist){
            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            if(stopDist-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param power of drive wheels
     */
    public boolean driveStraight(Directions direction, double power) {
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.FORWARD) {
            mDirection = -1;
        }
        power=power*mDirection;

        //converts current position into inches
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        if (cPosition < stopDist){
            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
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
     * @param direction "left" for left "right" for right
     */
    public boolean strafeSeconds(Directions direction){
        //sets direction strafing
        int mDirection=1;
        if(direction == Directions.RIGHT){
            mDirection=-1;
        }

        if (System.currentTimeMillis()<stopTime){
            fLeft.setPower(0.45*mDirection);
            fRight.setPower(-0.45*mDirection);
            bLeft.setPower(-0.45*mDirection);
            bRight.setPower(0.45*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * Sets end time for entire class' use
     * @param time
     */
    public void setTime (double time) {
        stopTime=time+System.currentTimeMillis();
    }

    /**
     * turns to the desired angle
     * 0-360 in a clockwise format
     * @param destination
     */
    public boolean rotation(double destination) {
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
        if (heading < destination - 0.5 || heading > destination + 0.5) {
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
        }
        else {
            fLeft.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            return true;
        }

        return false;
    }

    public boolean driveToLine(ColorSensor color, Directions dir){

        int direction = dir == Directions.FORWARD ? -1 : 1;

        if (color.red()<35&&color.blue()<35){
            fLeft.setPower(0.3*direction);
            bLeft.setPower(0.3*direction);
            bRight.setPower(0.3*direction);
            fRight.setPower(0.3*direction);
        }
        else{
            fLeft.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            return true;
        }

        return false;
    }

    public boolean strafeToLine(ColorSensor color, Directions dir){

        int direction = dir == Directions.RIGHT ? 1 : -1;

        if (color.red()<35&&color.blue()<35){
            fLeft.setPower(-0.6*direction);
            bLeft.setPower(0.6*direction);
            bRight.setPower(-0.6*direction);
            fRight.setPower(0.6*direction);
        }
        else {
            fLeft.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * turns to given angle without correction
     * @param distance angle destination
     * @param power drive power
     */
    public boolean simpleTurn(double distance,double power){
        telemetry.addData("angle",gyro.getRawYaw());
        if(distance<gyro.getRawYaw()) {
            telemetry.update();
            fLeft.setPower(-power);
            bLeft.setPower(-power);
            bRight.setPower(power);
            fRight.setPower(power);
        }
        else if (distance > gyro.getRawYaw()) {
            telemetry.update();
            fLeft.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(-power);
            fRight.setPower(-power);
        }
        else {
            fLeft.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * strafe to a given range.
     * Need to call sensors distanceR and distanceL
     * @param dSensor distance sensor object
     * @param distance distance driving to
     * @param sLocal location of sensor(center,right,left)
     */
    public boolean driveRange(DistanceSensor dSensor, double distance, Positions sLocal){
        double cDistance=0;
        if(dSensor.getDistance(DistanceUnit.CM)>150){
            cDistance=150;
        }
        else{
            cDistance=dSensor.getDistance(DistanceUnit.CM);
        }
        int dir;
        if(sLocal == Positions.CENTER) {
            if(distance<cDistance){
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
            else if(distance>cDistance) {
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
            if (distance < cDistance + 0.75 && distance > cDistance - 0.75) {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }
        else{
            if(sLocal == Positions.RIGHT){
                dir = 1;
            }
            else{
                dir = -1;
            }

            if (distance>dSensor.getDistance(DistanceUnit.CM)) {
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
            else if (distance<dSensor.getDistance(DistanceUnit.CM)) {
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
            if (distance < dSensor.getDistance(DistanceUnit.CM) + 0.75 && distance > dSensor.getDistance(DistanceUnit.CM) - 0.75) {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }

        return false;
    }

    /**
     * Waits until the set time passes
     */
    public boolean delay(){
        if (System.currentTimeMillis() >= stopTime) {
            return true;
        }

        return false;
    }


}
