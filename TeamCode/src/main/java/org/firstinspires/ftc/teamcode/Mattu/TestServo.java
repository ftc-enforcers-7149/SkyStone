package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestServo")
public class TestServo extends OpMode {

    Servo drive;

    boolean lastA, lastB;

    public void init() {
        drive = hardwareMap.servo.get("lArm");
    }

    public void loop() {
        if (gamepad1.a != lastA) {
            if (gamepad1.a) {
                if (drive.getPosition() <= 0.95) {
                    drive.setPosition(drive.getPosition() + 0.05);//0.9
                }
            }
        }
        if (gamepad1.b != lastB) {
            if (gamepad1.b) {
                if (drive.getPosition() >= 0.05) {
                    drive.setPosition(drive.getPosition() - 0.05);//0
                }
            }
        }

        telemetry.addData("Servo Position: ", drive.getPosition());

        lastA = gamepad1.a;
        lastB = gamepad1.b;
    }
}
