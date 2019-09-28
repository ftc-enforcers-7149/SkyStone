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


    //Class vars
    private double fDLast, fDCurrent;
    private boolean firstTest = true;


    //Used for encoders
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

    }

    /**
     * detects if object is moving or not
     * @param lastDTraveled
     * @param currentDTraveled
     * @param deltaTime
     * @return
     */
    public boolean isMoving(double lastDTraveled, double currentDTraveled, double deltaTime) {

        if(firstTest) {
            fDLast = distanceC.getDistance(DistanceUnit.CM);
            firstTest = false;
        }

        fDCurrent = distanceC.getDistance(DistanceUnit.CM);

        if(-(fDCurrent - fDLast)/deltaTime > (convertINtoCM(currentDTraveled) - convertINtoCM(lastDTraveled))/deltaTime + 0.2 ){
            return true;
        }

        return false;
    }

    /**
     * converts inches to cm. duh
     * @param input
     * @return
     */
    public double convertINtoCM(double input) {
       return input * 2.54;
    }

    public void handleMovement(String moveDir, double motorPower) {

        if(moveDir.equals("left")) {

        }

    }

}
