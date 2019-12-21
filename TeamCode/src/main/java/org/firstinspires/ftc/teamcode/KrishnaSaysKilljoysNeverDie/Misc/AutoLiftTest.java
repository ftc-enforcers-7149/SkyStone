package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "Auto Lift")
public class AutoLiftTest extends OpMode {

    //Drive train
    Headless driveSystem;

    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight,fLeft,bRight,bLeft, lift;

    DistanceSensor distanceLift;
    Gyroscope gyroscope;

    float armUp;
    float liftUp,liftDown;
    boolean lFoundationDown, rFoundationDown;
    float grab;
    boolean startAccel;

    boolean levelPlus, levelMinus, pressP, pressM,liftPress;

    int level;

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
        distanceLift = hardwareMap.get(DistanceSensor.class, "distanceLift");

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
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

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
        levelPlus = gamepad1.dpad_up;
        levelMinus = gamepad1.dpad_down;


        //Arms and block grabbers
        if (armUp > 0.1) {
            lArm.setPosition(0.95);
            rArm.setPosition(0.81);
        }
        else {
            lArm.setPosition(0.36); //0
            rArm.setPosition(0.3);  //0
        }

        if(grab > 0.1){
            rGrab.setPosition(.20);//.41
            lGrab.setPosition(0.28);//.45
        }
        else{
            rGrab.setPosition(.13);//.6
            lGrab.setPosition(0.21);//1
        }

        if(!pressP){
            if (levelPlus) {
                if (level < 8){
                    level++;
                    liftPress=true;
                }
                pressP = true;
            }

        }
        else{
            if (!levelPlus){
                pressP = false;
            }
        }

        if(!pressM){
            if (levelMinus) {
                if(level > 0){
                    level--;
                    liftPress=true;
                }
                pressM = true;

            }

        }
        else{
            if (!levelMinus){
                pressM = false;
            }
        }

        if(liftPress) {
            switch (level) {
                case 0:
                    if(distanceLift.getDistance(DistanceUnit.CM) > 3) { //3
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0);
                        liftPress = false;
                    }
                    break;
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 10) {  //10.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 11) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 20) { //20.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 21) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 30) { //30.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 31) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 40) { //40.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 41) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 55) {   //54
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 56) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 6:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 63) {   //62
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 64) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 7:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 73) {   //72
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 74) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 8:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 82) {   //81
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 83) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
            }
        }

        telemetry.addData("Level: ", level);
        telemetry.addData("Lift height: ", distanceLift.getDistance(DistanceUnit.CM));


    }
    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        lift.setPower(0);
    }

    public void goLevel(String dir) {
        if (dir.equals("up")) {
            switch (level) {
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM)< 10.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM)< 20) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM)< 30.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM)< 40.5) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
                case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM)< 54) {
                        lift.setPower(0.8);
                    }
                    lift.setPower(0);
                    break;
            }

        }
        if(dir.equals("down")) {
            switch (level) {
                case 0:
                    if(distanceLift.getDistance(DistanceUnit.CM) > 3) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 40.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 30.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 20) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 10.5) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;
                /*case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 3) {
                        lift.setPower(-0.4);
                    }
                    lift.setPower(0);
                    break;*/

            }
        }
    }

}
