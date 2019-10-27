package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="auto claw park")
public class AutoClawPark extends OpMode {
    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;

    public void init(){
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");

        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        lArm.setPosition(0.1);
        rArm.setPosition(0.05);
        //lGrab.setPosition(0.7);
        //rGrab.setPosition(0.25);
    }

    public void loop(){
        lArm.setPosition(0.65);
        rArm.setPosition(0.45);
        rGrab.setPosition(0.15);
        lGrab.setPosition(0.5);
    }
    public void stop(){

    }
}
