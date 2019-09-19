package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CollisionData extends OpMode {

    //DC Motors
DcMotor Motor1; //Left motor
DcMotor Motor2; //right motor

    //Distance Sensors


    public void loop() {
        Motor1.setPower(gamepad1.left_stick_x);
        Motor2.setPower(-gamepad1.left_stick_x);
    }

    public void init() {
        Motor1 = hardwareMap.dcMotor.get("m1");
        Motor2 = hardwareMap.dcMotor.get("m2");
    }

    public void stop() {

    }

}
