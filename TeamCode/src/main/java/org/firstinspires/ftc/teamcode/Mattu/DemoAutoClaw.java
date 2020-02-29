package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Demobot Claw")
public class DemoAutoClaw extends OpMode {

    Servo lArm, rArm;

    int step = 0;

    public void init() {
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);

        lArm.setPosition(0.6);
        rArm.setPosition(0.8);
    }

    public void start() {
        lArm.setPosition(0.15);
        rArm.setPosition(0.37);
    }

    public void loop() {
        telemetry.addData("Step: ", step);

        switch (step) {
            case 0:

                break;
        }
    }
}