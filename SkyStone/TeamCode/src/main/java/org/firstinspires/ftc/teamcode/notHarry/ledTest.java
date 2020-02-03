package org.firstinspires.ftc.teamcode.notHarry;

import android.graphics.Color;
import android.graphics.PixelFormat;
import android.provider.CalendarContract;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode._Reference.LEDTest;

@TeleOp(name = "LED Auto Lift")
public class ledTest extends OpMode {
    //Servo lArm, rArm, lGrab, rGrab;
    Telemetry.Item patternName;
    Telemetry.Item display;



    LEDTest.DisplayKind displayKind;


    //Drive train
    Headless driveSystem;

    Claw claw;

    //Hardware

    DcMotor fRight, fLeft, bRight, bLeft, lift;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    DistanceSensor distanceLift;
    Gyroscope gyroscope;

    float armUp;
    float liftUp, liftDown;
    boolean lFoundationDown, rFoundationDown;
    float grab;
    boolean startAccel;

    boolean levelPlus, levelMinus, pressP, pressM, liftPress;

    int level;

    public void init() {
        //Servos

        displayKind = LEDTest.DisplayKind.AUTO;


        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        /*lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);*/

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
        //lift = hardwareMap.dcMotor.get("lift");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize drive train
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);


        //Lift brake
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //claw = new Claw(lArm,rArm,lGrab,rGrab);


    }

    public void loop() {



        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(pattern);

        display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

        //Inputs
        armUp = gamepad2.left_trigger;
        grab = gamepad2.right_trigger;
        liftUp = gamepad1.right_trigger;
        liftDown = gamepad1.left_trigger;
        lFoundationDown = gamepad1.left_bumper || gamepad2.x;
        rFoundationDown = gamepad1.right_bumper || gamepad2.b;
        startAccel = gamepad1.x;
        levelPlus = gamepad1.y;
        levelMinus = gamepad1.a;


        if (distanceLift.getDistance(DistanceUnit.CM) <= 2) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 3.5 && distanceLift.getDistance(DistanceUnit.CM) < 7) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 12 && distanceLift.getDistance(DistanceUnit.CM) < 16) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 24 && distanceLift.getDistance(DistanceUnit.CM) < 28) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 36 && distanceLift.getDistance(DistanceUnit.CM) < 40) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 44 && distanceLift.getDistance(DistanceUnit.CM) < 48) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 56 && distanceLift.getDistance(DistanceUnit.CM) < 60) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 66 && distanceLift.getDistance(DistanceUnit.CM) < 70) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 77 && distanceLift.getDistance(DistanceUnit.CM) < 81) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (distanceLift.getDistance(DistanceUnit.CM) > 85 && distanceLift.getDistance(DistanceUnit.CM) < 89) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }

        /*if(grab > 0.1){
            claw.grab();
        }
        else{
            claw.release();
        }*/


        if (!pressP) {
            if (levelPlus) {
                if (level < 9) {
                    level++;
                    liftPress = true;
                }
                pressP = true;
            }

        } else {
            if (!levelPlus) {
                pressP = false;
            }
        }

        if (!pressM) {
            if (levelMinus) {
                if (level > 0) {
                    level--;
                    liftPress = true;
                }
                pressM = true;

            }

        } else {
            if (!levelMinus) {
                pressM = false;
            }
        }

        if (liftPress) {
            switch (level) {
                case 0:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 1.5) { //3
                        lift.setPower(-0.4);
                    } else {
                        lift.setPower(0);
                        liftPress = false;
                    }
                    break;
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 4) {  //10.5
                        lift.setPower(0.45);//0.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 7) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 13) { //20.5
                        lift.setPower(0.45);//.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 16) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 24) { //30.5
                        lift.setPower(0.45);//.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 26) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 35) { //40.5
                        lift.setPower(0.45);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 38) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 45.5) {   //54
                        lift.setPower(0.47);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 47.5) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 6:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 56) {   //62
                        lift.setPower(0.49);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 58) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 7:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 66) {   //62
                        lift.setPower(0.49);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 68) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 8:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 77) {   //72
                        lift.setPower(0.51);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 79) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 9:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 87) {   //81
                        lift.setPower(0.51);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 90) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;

            }
        }

        telemetry.addData("Level: ", level);
        telemetry.addData("Lift height: ", distanceLift.getDistance(DistanceUnit.CM));


    }

    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        lift.setPower(0);
    }
}