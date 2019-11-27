package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name="Blue SkyStone ParkV2")
public class BlueSkyStonePark extends OpMode {
    public Servo lArm, rArm, lGrab, rGrab;
    Servo fLFound, fRFound, bLFound, bRFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    int step=0;

    Webcam webcam;
    DriveTrain driveTrain;
    Claw claw;

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
        claw=new Claw(lArm,rArm,lGrab,rGrab);
    }
    public void start(){
        driveTrain=new DriveTrain(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
    }
    public void loop(){
        switch(step){
            case 0:
                driveTrain.driveStraight("forward",20,0.2);
                break;
            case 1:
                driveTrain.driveRange(distanceR,65,"right");
            case 2:
                webcam.captureFrameToFile();
                double iTime=System.currentTimeMillis();
                while(System.currentTimeMillis()<iTime+4000) {
                    position = webcam.getBitmapPos(telemetry);
                }
                webcam.deactivate();
                break;
            case 3:
                if(position.equals("right")){
                    driveTrain.driveRange(distanceR,56,"right");
                }
                else if(position.equals("left")){
                    driveTrain.driveRange(distanceR,92,"right");
                }
                else{
                    driveTrain.driveRange(distanceR,75,"right");
                }
                break;
            case 4:
                claw.down();
                claw.release();
                break;
            case 5:driveTrain.delay(500);
                break;
            case 6:
                driveTrain.driveStraight("forward", 25);
                break;//
            case 7:
                claw.grab();
                break;
            case 8:
                driveTrain.delay(500);
                break;
            case 9:
                claw.up();
                break;
            case 10:
               driveTrain.driveStraight("backward",17);
                break;
            case 11:
                driveTrain.rotation(90);
                break;
            case 12:
                claw.down();
                break;
            case 13:
                driveTrain.driveToLine(color, "blue", "forward");
                break;
            case 14:
                driveTrain.driveStraight("forward",20);
                break;
            case 15:
                claw.release();
                claw.up();
                break;
            case 16:
                driveTrain.driveToLine(color, "blue", "backward");
                break;
            case 17:
                driveTrain.rotation(80);
                break;
            case 18://
                driveTrain.driveRange(distanceC,40,"center");

        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceR.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }
    public void stop(){

    }
}
