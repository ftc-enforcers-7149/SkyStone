package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//Made by Krishna and Nathanael, with help from Matteo
//Made 9-21-19
public class CollisionData extends OpMode {

    //Mr. Worldwide but variables


    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //Distance Sensors
    DistanceSensor front, distBLeft, distBRight;


    //Speed related vars (Speed, set speed (buttons))
    float speedTwoThirds, speedFull;
    boolean speedHalf;
    double speed = 1;

    //Testing buttons
    boolean startTest, testing, endTest;


    //Time vars
    double sTime, cTime;


    //Delta vars
    double delta1, delta2, delta3, delta4;
    boolean firstTest, secondTest, thirdTest;
    String output;


    //Dist and motor vars
    double fRightPower, fDist, bLDist, bRDist;

    public void init() {

        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        front = hardwareMap.get(DistanceSensor.class, "front");
        distBLeft = hardwareMap.get(DistanceSensor.class, "distBLeft");
        distBRight = hardwareMap.get(DistanceSensor.class, "distBRight");

        testing = false;
        firstTest = true;
        delta1 = 0; delta2 = 0; delta3 = 0; delta4 = 0;

    }

    public void loop() {

        //Buttons
        speedFull = gamepad1.left_trigger;
        speedTwoThirds = gamepad1.right_trigger;
        speedHalf = gamepad1.a;
        startTest = gamepad1.b;
        endTest = gamepad1.x;


        //Variables
        fRightPower = fRight.getPower();
        fDist = front.getDistance(DistanceUnit.CM);
        bLDist = distBLeft.getDistance(DistanceUnit.CM);
        bRDist = distBRight.getDistance(DistanceUnit.CM);


        //Telemetry
        telemetry.addData("Motor speed: ", fRight.getPower());
        telemetry.addData("Front Dist: ", fDist);
        telemetry.addData("Back L Dist: ", bLDist);
        telemetry.addData("Back R Dist: ", bRDist);


        //Set speed
        if(!testing) {
            if(speedFull > 0.1) {
                speed = 1;
            }
            else if(speedHalf) {
                speed = 0.5;
            }
            else if(speedTwoThirds > 0) {
                speed = 0.67;
            }
        }


        //Test started?
        if(startTest) {
            sTime = System.currentTimeMillis();
            Log.w("Speed: ", Double.toString(speed));
            testing = true;
        }


        //Testing process
        if(testing) {

            //Starts moving bot
            if(gamepad1.left_stick_y > 0 || gamepad1.right_stick_y > 0) {
                //Drive
                fLeft.setPower(speed);
                fRight.setPower(speed);
                bLeft.setPower(speed);
                bRight.setPower(speed);
            }

            //Ends test
            if(cTime - sTime > 5000 || endTest) {
                testing = false;
            }

            //Testing
            if(firstTest) {
                delta1 = (fDist - 0)/2;
                secondTest = true;
                firstTest = false;
            }
            else if(secondTest) {
                delta2 = delta1;
                delta1 = fDist - delta2;
                thirdTest = true;
                secondTest = false;
            }
            else if(thirdTest) {
                delta3 = delta2;
                delta2 = delta1;
                delta1 = fDist - delta2;
                thirdTest = false;
            }
            else {
                delta4 = delta3;
                delta3 = delta2;
                delta2 = delta1;
                delta1 = fDist - delta2;
            }

            //Logs delta
            Log.i("Delta: ", Double.toString(delta1));
            Log.i("Time: ", Double.toString(cTime/1000));
        }

    }

    public void stop() {
        //Stops the program duh
        //Why do you even need a comment for this part of the code smh
        //There's not even anything here
        //It's eeeeeeeemptttyyyyyy
        //This code empty YEET
    }

}
