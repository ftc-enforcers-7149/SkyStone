package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class CollisionData extends OpMode {

    //DC Motors
DcMotor motor1, motor2; //Left and right motors
    //Distance Sensors
    public void init() {
        motor1 = hardwareMap.dcMotor.get("m1");
        motor2 = hardwareMap.dcMotor.get("m2");
    }

    public void loop() {
        motor1.setPower(gamepad1.left_stick_x);
        motor2.setPower(-gamepad1.left_stick_x);
    }

    public void stop() {

    }

}
