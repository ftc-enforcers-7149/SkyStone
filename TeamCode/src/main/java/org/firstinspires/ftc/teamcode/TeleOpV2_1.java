package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

//@TeleOp(name = "TeleOp v2")
public class TeleOpV2_1 extends OpMode {
    //Drive train
    Headless driveSystem;

    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft, liftMotor;
    DistanceSensor distanceLift;

    Claw claw;
    FoundationV2 foundation;
    Lift lift;

    float armUp;
    boolean isBreak=false;
    float liftUp,liftDown;
    boolean lFoundationDown, rFoundationDown;
    float grab;
    boolean startAccel;


    public void init(){
        //Servos
        fLFound=hardwareMap.servo.get("fLFound");
        fRFound=hardwareMap.servo.get("fRFound");
        bLFound=hardwareMap.servo.get("bLFound");
        bRFound=hardwareMap.servo.get("bRFound");
        lArm=hardwareMap.servo.get("lArm");
        rArm=hardwareMap.servo.get("rArm");
        lGrab=hardwareMap.servo.get("lGrab");
        rGrab=hardwareMap.servo.get("rGrab");

        //Inits to combat lag
        /*colorSensor = hardwareMap.colorSensor.get("color");
        distL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distC = hardwareMap.get(DistanceSensor.class, "distanceC");*/

        distanceLift=hardwareMap.get(DistanceSensor.class,"distanceLift");

        //Drive motors
        fLeft=hardwareMap.dcMotor.get("fLeft");
        fRight=hardwareMap.dcMotor.get("fRight");
        bLeft=hardwareMap.dcMotor.get("bLeft");
        bRight=hardwareMap.dcMotor.get("bRight");
        liftMotor=hardwareMap.dcMotor.get("lift");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize drive train
        Gyroscope gyroscope=new Gyroscope(telemetry,hardwareMap);
        driveSystem=new Headless(gyroscope,fLeft,fRight,bLeft,bRight);

        //Servo directions
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        //Lift brake
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundation=new FoundationV2(fLFound,fRFound,bLFound,bRFound);
        claw=new Claw(lArm,rArm,lGrab,rGrab);
        lift=new Lift(liftMotor,distanceLift);
    }

    public void loop(){
        //Inputs
        armUp = gamepad2.left_trigger;
        grab = gamepad2.right_trigger;
        liftUp=gamepad1.right_trigger;
        liftDown=gamepad1.left_trigger;
        lFoundationDown = gamepad1.left_bumper || gamepad2.x;
        rFoundationDown = gamepad1.right_bumper || gamepad2.b;
        startAccel = gamepad1.x;


        //Drive
        driveSystem.drive(gamepad1);

        //FoundationV1 grabbers
        if (lFoundationDown) {
            foundation.lDown();
        }
        else {
            foundation.lUp();
        }

        if (rFoundationDown) {
            foundation.rDown();
        }
        else {
            foundation.rUp();
        }

        //Arms and block grabbers
        if (armUp > 0.1) {
            claw.up();
        }
        else {

            claw.down();
        }

        if(grab > 0.1){
            claw.grab();
        }
        else{
            claw.release();
        }

        //Lift
        lift.liftSet(gamepad1);

        /*if(liftUp>0.1){
            liftMotor.setPower(0.8);
            isBreak=true;
        }
        else if(liftDown>0.1){
            liftMotor.setPower(-0.4);
            isBreak=false;
        }
        else{
            liftMotor.setPower(0);
        }*/

        if (startAccel) {
            driveSystem.setAccel();
        }

        telemetry.addData("Lift Level", lift.getLevel());
    }

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        liftMotor.setPower(0);
    }
}
