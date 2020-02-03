package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

//@Autonomous(name="Red SkyStone ParkV2")
public class RedSkyStoneParkV2 extends OpMode {
    public Servo lArm, rArm, lGrab, rGrab;
    Servo fLFound, fRFound, bLFound, bRFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    int step=0;

    Webcam webcam;
    DriveTrainV1 driveTrainV1;

    String position="";

    //Distance Sensors
    DistanceSensor distanceL, distanceR, distanceC;
    ColorSensor color;
    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");

        color = hardwareMap.get(ColorSensor.class, "color");

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lArm.setPosition(0.95);
        rArm.setPosition(0.81);

        fLFound.setPosition(1);
        bLFound.setPosition(1);

        fRFound.setPosition(1);
        bRFound.setPosition(1);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("sensor:",distanceC.getDeviceName());

        webcam=new Webcam(hardwareMap);
    }
    public void start(){
        //driveTrain =new DriveTrainV1(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
    }
    public void loop(){
        switch(step){
            case 0:
                driveTrainV1.driveStraight("forward",20);
                break;
            case 1:
                double startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() < startTime + 4000) {
                    position = webcam.getBitmapPos(telemetry);
                }
                webcam.deactivate();
                break;
            case 2:
                if(position.equals("right")){
                    driveTrainV1.driveRange(distanceL,106,"left");
                }
                else if(position.equals("left")){
                    driveTrainV1.driveRange(distanceL,75,"left");
                }
                else{
                    driveTrainV1.driveRange(distanceL,90,"left");
                }
                break;
            case 3:
                lArm.setPosition(0);
                rArm.setPosition(0);
                lGrab.setPosition(1);
                rGrab.setPosition(0.43);
                break;

            case 4:
                driveTrainV1.delay(500);
                break;
            case 5:
                driveTrainV1.driveStraight("forward", 25);
                break;
            case 6:
                lGrab.setPosition(0.45);
                rGrab.setPosition(0.43);
                break;
            case 7:
                driveTrainV1.delay(500);
                break;
            case 8:
                lArm.setPosition(0.95);
                rArm.setPosition(0.81);
                break;
            case 9:
               driveTrainV1.driveStraight("backward",17);
                break;
            case 10:
                driveTrainV1.rotation(272);
                break;
            case 11:
                lArm.setPosition(0);
                rArm.setPosition(0);
                break;
            case 12:
                driveTrainV1.driveToLine(color, "red", "forward");
                break;
            case 13:
                driveTrainV1.driveStraight("forward",20);
                break;
            case 14:
                rGrab.setPosition(0.6);
                lGrab.setPosition(1);
                break;
            case 15:
                driveTrainV1.driveToLine(color, "red", "backward");
                lArm.setPosition(0.95);
                rArm.setPosition(0.81);
                break;
            case 16:
                //driveTrain.simpleTurn(0,0.4);
                break;

        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }
    public void stop(){

    }
}
