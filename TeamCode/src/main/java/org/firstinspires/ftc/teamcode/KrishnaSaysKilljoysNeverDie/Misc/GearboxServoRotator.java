package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name="GearboxServoRotator")
public class GearboxServoRotator extends OpMode {

    Servo drive;

    public void init() {
        drive = hardwareMap.servo.get("drive");

    }

    public void loop() {
        if(gamepad1.a) {
            drive.setPosition(.6);
        }
        else {
            drive.setPosition(0);
        }
    }

}
