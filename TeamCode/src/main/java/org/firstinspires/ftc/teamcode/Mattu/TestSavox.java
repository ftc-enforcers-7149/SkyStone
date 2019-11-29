package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Savox")
public class TestSavox extends OpMode {

    Servo testServo;

    public void init() {
        testServo = hardwareMap.servo.get("test");
        testServo.scaleRange(0.0012, 1);
    }

    public void loop() {
        if (gamepad1.a) {
            testServo.setPosition(1);
        }
        if (gamepad1.b) {
            testServo.setPosition(0);
        }

        telemetry.addData("Servo Pos: ", testServo.getPosition());
    }
}
