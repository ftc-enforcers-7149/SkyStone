package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MovementDetectionClass{

    //Declarations of hardware
    DistanceSensor distanceL, distanceC, distanceR;
    DcMotor fLeft, fRight, bLeft, bRight;

    //fDLast fDCurrent front distances
    //lDLast lDCurrent left distances
    //rDLast rDCurrent right distances
    private double fDLast, fDCurrent;
    private double lDLast, lDCurrent;
    private double rDLast, rDCurrent;
    private double tDLast, tDCurrent;

    //Used for  encoders
    static final double     EXTERNAL_GEARING        = 1;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;


    /**
     *
     * @param distanceC distC
     * @param distanceR distR
     * @param distanceL distR
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     */
    public MovementDetectionClass(DistanceSensor distanceC, DistanceSensor distanceR, DistanceSensor distanceL, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight) {
        //Mapping distance sensors
        this.distanceC = distanceC;
        this.distanceR = distanceR;
        this.distanceL = distanceL;

        //Mapping motors
        this.fLeft= fLeft;
        this.fRight = fRight;
        this.bRight = bRight;
        this.bLeft = bLeft;

        //Brakes
        this.fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoderWithoutEncoder();

        fDCurrent = distanceC.getDistance(DistanceUnit.CM);
        lDCurrent = distanceL.getDistance(DistanceUnit.CM);
        rDCurrent = distanceR.getDistance(DistanceUnit.CM);
        tDCurrent = 0;
    }

    /**
     * returns center range center distance
     * @return distance in cm
     */
    public double getCDistance(){
        return distanceC.getDistance(DistanceUnit.CM);
    }

    /**
     * returns right range center distance
     * @return distance in cm
     */
    public double getRDistance(){
        return distanceR.getDistance(DistanceUnit.CM);
    }

    /**
     * returns left range center distance
     * @return distance in cm
     */
    public double getLDistance(){
        return distanceL.getDistance(DistanceUnit.CM);
    }

    public double getChange(String sensor) {
//
        double difference = 0;

        if (sensor.equals("front")) {
            difference = fDLast - fDCurrent;
        }
        else if (sensor.equals("left")) {
            difference = lDLast - lDCurrent;
        }
        else if (sensor.equals("right")) {
            difference = rDLast - rDCurrent;
        }

        return difference;
    }

    /**
     * Detects if object is moving or not in front
     * @return
     */
    public boolean isFrontMoving() {
        if (getChange("front") > (tDCurrent - tDLast) + 3) {
            return true;
        }

        return false;
    }

    /**
     * Detects if obstacle on front is close
     * @return
     */
    public boolean isFrontClose() {
        if (fDCurrent < 20) {
            return true;
        }

        return false;
    }

    /**
     * Detects if obstacle on left is close
     * @return
     */
    public boolean isLeftClose() {
        if (lDCurrent < 20) {
            return true;
        }

        return false;
    }

    /**
     * Detects if obstacle on right is close
     * @return
     */
    public boolean isRightClose() {
        if (rDCurrent < 20) {
            return true;
        }

        return false;
    }

    public void update() {
        tDLast = tDCurrent;
        fDLast = fDCurrent;
        rDLast = rDCurrent;
        lDLast = lDCurrent;

        lDCurrent = distanceL.getDistance(DistanceUnit.CM);
        rDCurrent = distanceR.getDistance(DistanceUnit.CM);
        tDCurrent = convertINtoCM(fLeft.getCurrentPosition() / COUNTS_PER_INCH);
        fDCurrent = distanceC.getDistance(DistanceUnit.CM);
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
     * converts inches to cm.
     * @param input
     * @return
     */
    public double convertINtoCM(double input) {
       return input * 2.54;
    }

    /**
     * Returns one string containing current and last travel and sensor distances
     * @return
     */
    public String rawData() {
        return "Distance traveled: " + tDCurrent + ", Last distance traveled: " + tDLast +
                "\nSensor Distance: " + fDCurrent + ", Last sensor distance: " + fDLast;
    }
}
