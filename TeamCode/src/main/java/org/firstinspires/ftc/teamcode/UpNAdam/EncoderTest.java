package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

//@TeleOp(name = "Encoder Testing")
public class EncoderTest extends OpMode {
    //Drive train
    Headless driveSystem;

    //O do met ry
    OdometryPosition oP;
    Range range;

    DcMotor fRight, fLeft, bRight, bLeft;

    //Declaring motors
    DcMotor encoderY, encoderX;

    Gyroscope gyroscope;

    ColorSensor color;

    double positionY,positionX;

    double yDisp;
    double xDisp;

    boolean startAccel;
    boolean isTurning = false;

    private static final double     EXTERNAL_GEARING        = 1.5;    //From sprockets
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    //used for encoders (y)
    private static final double     COUNTS_PER_MOTOR_REVY    = 1440;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESY  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHY        = COUNTS_PER_MOTOR_REVY /(WHEEL_DIAMETER_INCHESY * Math.PI);

    //used for encoders (x)
    private static final double     COUNTS_PER_MOTOR_REVX    = 1440;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESX  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHX      = COUNTS_PER_MOTOR_REVX /(WHEEL_DIAMETER_INCHESX * Math.PI);

    public void init(){
//Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        encoderX = hardwareMap.dcMotor.get("encX");
        encoderY = hardwareMap.dcMotor.get("encY");


        color = hardwareMap.colorSensor.get("color");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderY.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroscope = new Gyroscope(telemetry, hardwareMap);
        //Initialize drive train
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

        range = new Range(hardwareMap, "distanceL", "distanceR", "distanceC");


        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        driveSystem.drive(gamepad1);
        telemetry.addData("Red: ", color.red());
        telemetry.addData("left: ", range.getLeft(DistanceUnit.INCH));

        if (gamepad1.a) {
            isTurning = true;
        } else if (gamepad1.b){
            isTurning = false;

            encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Sets our encoders to run again
            encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
        //Gets our heading and change in x and y odometers
        double heading = gyroscope.getRawYaw();
        yDisp = encoderY.getCurrentPosition();
        xDisp = encoderX.getCurrentPosition();

        //When turning, the odometers are not accurate, so they must be ignored
        if (!isTurning) {
            //Converts the robot's angle for use with sine and cosine
            //Then uses that as a modifier for how much an odometer will effect that axis

            //Apply the x odometer to the x and y axes
            positionX = ((xDisp/COUNTS_PER_INCHX) * Math.sin(Math.toRadians(gyroscope.cvtTrigAng(heading)))) - ((yDisp/COUNTS_PER_INCHY)* Math.cos(Math.toRadians(gyroscope.cvtTrigAng(heading))));

            //Apply the y odometer to the x and y axes
            positionY = ((yDisp/COUNTS_PER_INCHY) * Math.sin(Math.toRadians(gyroscope.cvtTrigAng(heading)))) + ((xDisp/COUNTS_PER_INCHX) * Math.cos(Math.toRadians(gyroscope.cvtTrigAng(heading))));
        }

        //Rounds the positions so you don't get numbers like 6.6278326e^-12678
        positionY = Math.ceil(positionY * 10000) / 10000;
        positionX = Math.ceil(positionX * 10000) / 10000;

        double relativeY = fLeft.getPower() + bLeft.getPower() + fRight.getPower() + bRight.getPower();
        double relativeX = fLeft.getPower() - bLeft.getPower() - fRight.getPower() + bRight.getPower();
        double robotAngle = Math.atan2(relativeY, relativeX) - Math.toRadians(gyroscope.cvtTrigAng(heading)) + Math.PI / 4;

        telemetry.addData("posX:",positionX);//telemetry.addData("posX:",positionX);
        telemetry.addData("posY:",positionY);
        telemetry.addData("raw x: ", encoderX.getCurrentPosition());
        telemetry.addData("raw y: ", encoderY.getCurrentPosition());
        telemetry.addData("heading:" ,heading);
        telemetry.addLine();
        telemetry.addData("fLeft: ", fLeft.getPower());
        telemetry.addData("fRight: ", fRight.getPower());
        telemetry.addData("bLeft: ", bLeft.getPower());
        telemetry.addData("bRight: ", bRight.getPower());

        RobotLog.vv("ODOMETRY", "Pos X: " + positionX + " Pos Y: " + positionY);
    }

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
