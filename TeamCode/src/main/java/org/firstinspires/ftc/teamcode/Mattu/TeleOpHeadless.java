package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Headless;

@TeleOp(name = "Headless")
public class TeleOpHeadless extends OpMode {

    Headless driveSystem;

    public void init() {
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }

    public void loop() {
        driveSystem.drive(gamepad1);
    }
}
