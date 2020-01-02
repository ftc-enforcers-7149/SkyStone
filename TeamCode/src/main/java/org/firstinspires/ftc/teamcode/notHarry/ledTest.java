package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "LED Auto Lift")
public class ledTest extends OpMode {



        //Drive train
        Headless driveSystem;

        //Hardware

        DcMotor fRight,fLeft,bRight,bLeft, lift;

        RevBlinkinLedDriver blinkinLedDriver;
        RevBlinkinLedDriver.BlinkinPattern pattern;

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


            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

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



            //Lift brake
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
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

            if (distanceLift.getDistance(DistanceUnit.CM) <= 3) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            } else if (distanceLift.getDistance(DistanceUnit.CM) > 10 && distanceLift.getDistance(DistanceUnit.CM) < 11) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            }else if (distanceLift.getDistance(DistanceUnit.CM) > 20 && distanceLift.getDistance(DistanceUnit.CM) < 21) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
            }else if (distanceLift.getDistance(DistanceUnit.CM) > 30 && distanceLift.getDistance(DistanceUnit.CM) < 31) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            }else if (distanceLift.getDistance(DistanceUnit.CM) > 40 && distanceLift.getDistance(DistanceUnit.CM) < 41) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.GRAY;
            }else{
                pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
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


