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

    //Constructor
    MovementDetectionClass(HardwareMap hardwareMap, String distC, String distR, String distL, String fL, String bL, String fR, String bR) {
        //Mapping distance sensors
        distanceC = hardwareMap.get(DistanceSensor.class, distC);
        distanceR = hardwareMap.get(DistanceSensor.class, distR);
        distanceL = hardwareMap.get(DistanceSensor.class, distL);

        //Mapping motors
        fLeft= hardwareMap.dcMotor.get(fL);
        fRight = hardwareMap.dcMotor.get(fR);
        bRight = hardwareMap.dcMotor.get(bR);
        bLeft = hardwareMap.dcMotor.get(bL);

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

        //Brakes
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoderWithoutEncoder();

        fDCurrent = distanceC.getDistance(DistanceUnit.CM);
        lDCurrent = distanceL.getDistance(DistanceUnit.CM);
        rDCurrent = distanceR.getDistance(DistanceUnit.CM);
        tDCurrent = 0;
    }

    public double getChange(String sensor) {

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

    public boolean isLeftClose() {
        if (lDCurrent < 20) {
            return true;
        }

        return false;
    }

    public boolean isRightClose() {
        if (rDCurrent < 20) {
            return true;
        }

        return false;
    }

    /**
     * Detects if object is moving or not on left
     * Only when robot is still
     * @return
     */
    public boolean isLeftMoving() {
        if (-2 < tDCurrent - tDLast && tDCurrent - tDLast < 2) {
            if (getChange("left") > (tDCurrent - tDLast) + 2) {
                return true;
            }
        }

        return false;
    }

    /**
     * Detects if object is moving or not on right
     * Only when robot is still
     * @return
     */
    public boolean isRightMovement() {
        if (-2 < tDCurrent - tDLast && tDCurrent - tDLast < 2) {
            if (getChange("right") > (tDCurrent - tDLast) + 2) {
                return true;
            }
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
