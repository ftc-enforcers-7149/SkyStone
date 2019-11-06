package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;

import java.util.Locale;

public class DriveTrain {
    private DcMotor fLeft, fRight, bLeft, bRight;
    //IMU variables
    private BNO055IMU imu;
    private Orientation angles;

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
     * @param hardwareMap hardwareMap
     * @param telemetry telemetry
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     */
    public DriveTrain(HardwareMap hardwareMap , Telemetry telemetry, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight){
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.telemetry = telemetry;

        this.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
    }

    /**
     * Main constructor
     * @param hardwareMap hardwareMap
     * @param telemetry telemetry
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     */
    public DriveTrain(HardwareMap hardwareMap , Telemetry telemetry, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight, MovementDetectionClass detection){
        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        this.telemetry = telemetry;

        this.telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
        this.detection=detection;
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
                power=0.2*mDirection;
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
        double min = 0.2;
        double max = 0.8;
        double iTime=System.currentTimeMillis();

        //standard current angle
        double heading = cvtDegrees(angles.firstAngle);

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
        while (heading < destination - 2 || heading > destination + 2) {
            telemetry.addData("heading",heading);
            telemetry.addData("speed",speed);
            telemetry.update();
            double delta = destination-heading; //the difference between destination and heading
            heading = cvtDegrees(angles.firstAngle);
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

    /**
     * turns to given angle without correction
     * @param distance angle destination
     * @param power drive power
     */
    public void simpleTurn(double distance,double power){
        telemetry.addData("angle",angles.firstAngle);
        if(distance<0){
            while(distance<angles.firstAngle){
                telemetry.update();
                fLeft.setPower(power);
                bLeft.setPower(power);
                bRight.setPower(-power);
                fRight.setPower(-power);
            }
        }
        else{
            while(distance>angles.firstAngle){
                telemetry.update();
                fLeft.setPower(-power);
                bLeft.setPower(-power);
                bRight.setPower(power);
                fRight.setPower(power);
            }
        }
    }

    /**
     * converts gyro degrees from -180 to 180 to be 0 to 360
     * @param heading
     * @return
     */
    public double cvtDegrees(double heading) {
        if (heading <0 ) {
            return 360 + heading;
        } else {
            return heading;
        }
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

    /**
     * method needed for gyro
     * @param angleUnit
     * @param angle
     * @return
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    /**
     * method needed for gyro
     * @param degrees
     * @return
     */
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
