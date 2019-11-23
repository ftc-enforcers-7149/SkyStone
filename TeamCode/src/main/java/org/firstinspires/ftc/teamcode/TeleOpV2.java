package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;

@TeleOp(name = "TeleOp v2")
public class TeleOpV2 extends OpMode {
    //Drive train
    Headless driveSystem;

    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft, lift;

    //Prevents lag in color sensor?
    /*ColorSensor colorSensor;
    DistanceSensor distL, distR, distC;*/

    //
    float armUp;
    boolean isBreak=false;
    float liftUp,liftDown;
    boolean lFoundationDown, rFoundationDown;
    float grab;
    boolean startAccel;

    public void init(){
        //Servos
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        //Inits to combat lag
        /*colorSensor = hardwareMap.colorSensor.get("color");
        distL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distC = hardwareMap.get(DistanceSensor.class, "distanceC");*/

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight);

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
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            fLFound.setPosition(0.46);
            bLFound.setPosition(0.46);
        }
        else {
            fLFound.setPosition(1);
            bLFound.setPosition(1);
        }

        if (rFoundationDown) {
            fRFound.setPosition(0.40);
            bRFound.setPosition(0.40);
        }
        else {
            fRFound.setPosition(1);
            bRFound.setPosition(1);
        }

        //Arms and block grabbers
        if (armUp > 0.1) {
            lArm.setPosition(0.95);
            rArm.setPosition(0.81);
        }
        else {

            lArm.setPosition(0);
            rArm.setPosition(0);
        }

        if(grab > 0.1){
            rGrab.setPosition(0.1);//.41
            lGrab.setPosition(0.5);//.45
        }
        else{
            rGrab.setPosition(0.25);//.6
            lGrab.setPosition(1);//1
        }

        /*if(leftG>0.1){
            lGrab.setPosition(0.45);
        }
        else{
           lGrab.setPosition(1);
        }*/

        //Lift
        if(liftUp>0.1){
            lift.setPower(0.8);
            isBreak=true;
        }
        else if(liftDown>0.1){
            lift.setPower(-0.4);
            isBreak=false;
        }
        else{
            lift.setPower(0);
        }
        /*else{
            if(isBreak){
                lift.setPower(0.3);
            }
            else{
                lift.setPower(0.0);
            }
        }*/

        if (startAccel) {
            driveSystem.setAccel();
        }
        else {
            driveSystem.setLim(isBreak);
        }

        telemetry.addData("fL servo pos: ", fLFound.getPosition());
        telemetry.addData("fR servo pos: ", fRFound.getPosition());
        telemetry.addData("bL servo pos: ", bLFound.getPosition());
        telemetry.addData("bR servo pos: ", bRFound.getPosition());


    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
