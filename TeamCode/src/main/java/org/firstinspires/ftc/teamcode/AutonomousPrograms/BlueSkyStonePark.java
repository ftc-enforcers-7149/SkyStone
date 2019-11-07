package org.firstinspires.ftc.teamcode.AutonomousPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name="Blue SkyStone Park")
public class BlueSkyStonePark extends OpMode {
    public Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    int step=0;

    Webcam webcam;
    DriveTrain driveTrain;

    String position="";

    //Distance Sensors
    DistanceSensor distanceL, distanceR, distanceC;
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

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        //Servos up
        rFound.setPosition(0);
        lFound.setPosition(0);

        lArm.setPosition(0.1);
        rArm.setPosition(0.05);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        webcam=new Webcam(hardwareMap);
    }
    public void start(){
        driveTrain=new DriveTrain(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
    }
    public void loop(){
        switch(step){
            case 0:
                driveTrain.driveStraight("forward",19);
                break;
            case 1:
                position=webcam.getPosition();
                break;
            case 2:
                if(position.equals("right")){
                    driveTrain.strafeRange(distanceR,68);
                }
                else if(position.equals("left")){
                    driveTrain.strafeRange(distanceR,110);
                }
                else{
                    driveTrain.strafeRange(distanceR,90);
                }
                break;
            case 3:
                rArm.setPosition(0.45);
                lArm.setPosition(0.65);
                lGrab.setPosition(0.7);
                rGrab.setPosition(0.5);
                break;
            case 4:
                driveTrain.driveStraight("forward", 17);
            case 5:
                lGrab.setPosition(0.47);
                rGrab.setPosition(0.42);
        }
        step++;
        telemetry.addData("position",position);
        telemetry.addData("Step: ", step);
    }
    public void stop(){

    }
}
