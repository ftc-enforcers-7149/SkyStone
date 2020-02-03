package org.firstinspires.ftc.teamcode.AutonomousPrograms.V1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

//@Autonomous(name="Blue SkyStone Park")
@Disabled
public class BlueSkyStoneParkOldV1 extends OpMode {
    public Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
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
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");
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
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.FORWARD);
        rGrab.setDirection(Servo.Direction.REVERSE);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        //Servos up
        rFound.setPosition(0);
        lFound.setPosition(0);

        lArm.setPosition(0.1);
        rArm.setPosition(0.05);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("sensor:",distanceC.getDeviceName());

        webcam=new Webcam(hardwareMap);
    }
    public void start(){
        //driveTrainV1 =new DriveTrainV1(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
    }
    public void loop(){
        switch(step){
            case 0:
                driveTrainV1.driveStraight("forward",17);
                break;
            case 1:
                position=webcam.getPosition();
                break;
            case 2:
                if(position.equals("right")){
                    driveTrainV1.driveRange(distanceR,60,"right");
                }
                else if(position.equals("left")){
                    driveTrainV1.driveRange(distanceR,90,"right");
                }
                else{
                    driveTrainV1.driveRange(distanceR,70,"right");
                }
                break;
            case 3:
                lArm.setPosition(1);
                rArm.setPosition(1);
                lGrab.setPosition(0);
                rGrab.setPosition(0.15);

            case 4:
                driveTrainV1.delay(500);
                break;
            case 5:
                driveTrainV1.driveStraight("forward", 25);
                break;
            case 6:
                lGrab.setPosition(0.16);
                rGrab.setPosition(0.16);
                break;
            case 7:
                driveTrainV1.delay(500);
                break;
            case 8:
                lArm.setPosition(0.25);
                rArm.setPosition(0.25);
                break;
            case 9:
                driveTrainV1.driveStraight("backward",17);
                break;
            case 10:
                driveTrainV1.rotation(83);
                break;
            case 11:
                lArm.setPosition(1);
                rArm.setPosition(1);
            case 12:
                //driveTrainV1.driveToLine(color);
                break;
            case 13:
                driveTrainV1.driveStraight("forward",28);
                break;
            case 14:
                lGrab.setPosition(0);
                rGrab.setPosition(0);
                break;
            case 15:
                driveTrainV1.driveRange(distanceC,20,"center");
                lArm.setPosition(0.25);
                rArm.setPosition(0.25);
                break;
            case 16:
                driveTrainV1.simpleTurn(0,0.4);
                break;

        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("range",distanceC.getDistance(DistanceUnit.CM));
        telemetry.addData("Step: ", step);
    }
    public void stop(){

    }
}
