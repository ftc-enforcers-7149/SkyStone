package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

public class DemoBotClawTest extends OpMode {
    Servo lGrab, rGrab,lArm,rArm;
    Claw claw;
    public void init(){
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");

        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);

        claw = new Claw(lArm,rArm,lGrab,rGrab);
    }

    public void loop(){

    }

}
