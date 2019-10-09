package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Tank;

@TeleOp(name = "Tank")
public class TeleOpTank extends OpMode {

    Tank driveSystem;

    public void init() {
        driveSystem = new Tank(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }

    public void loop() {
        driveSystem.drive(gamepad1);
    }
}
