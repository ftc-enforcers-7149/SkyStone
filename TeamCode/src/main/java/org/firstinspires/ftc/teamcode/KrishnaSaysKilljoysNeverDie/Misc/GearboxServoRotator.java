package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name="GearboxServoRotator")
public class GearboxServoRotator extends OpMode {

    Servo drive;

    double initialPosition = .90;
    double grabPosition = .66;

    public void init() {
        drive = hardwareMap.servo.get("drive");
    }

    public void loop() {

        if(gamepad1.a) {
            drive.setPosition(grabPosition);
        }
        else {
            drive.setPosition(initialPosition);
        }


        telemetry.addData("Initial pos: ", initialPosition);
        telemetry.addData("Grab pos: ", grabPosition);

    }

}
