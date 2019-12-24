package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@TeleOp(name = "Encoder Testing")
public class EncoderTest extends OpMode {
    //Drive train
    Headless driveSystem;

    //O do met ry
    OdometryPosition oP;

    DcMotor fRight, fLeft, bRight, bLeft;

    //Declaring motors
    DcMotor encoderY, encoderX;

    Gyroscope gyroscope;

    double positionY,positionX;

    double yDisp;
    double xDisp;

    boolean startAccel;
    boolean isTurning = false;

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

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        encoderY.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroscope = new Gyroscope(telemetry, hardwareMap);
        //Initialize drive train
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets our encoders to run again
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(){
        driveSystem.drive(gamepad1);

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
            positionX = ((xDisp/COUNTS_PER_INCHX) * Math.sin(Math.toRadians(gyroscope.cvtTrigAng(heading)))) + ((yDisp/COUNTS_PER_INCHY)* Math.cos(Math.toRadians(gyroscope.cvtTrigAng(heading))));

            //Apply the y odometer to the x and y axes
            positionY = ((yDisp/COUNTS_PER_INCHY) * Math.sin(Math.toRadians(gyroscope.cvtTrigAng(heading)))) + ((xDisp/COUNTS_PER_INCHX) * Math.cos(Math.toRadians(gyroscope.cvtTrigAng(heading))));
        }

        //Rounds the positions so you don't get numbers like 6.6278326e^-12678
        positionY = Math.ceil(positionY * 10000) / 10000;
        positionX = Math.ceil(positionX * 10000) / 10000;

        telemetry.addData("posX:",positionX);
        telemetry.addData("posY:",(positionY));
        telemetry.addData("isTurning:",isTurning);
        telemetry.addData("heading:" ,heading);
        telemetry.addData("cos: ",Math.cos(Math.toRadians(gyroscope.cvtTrigAng(heading))));
        telemetry.addData("sin: ",Math.sin(Math.toRadians(gyroscope.cvtTrigAng(heading))));

    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
