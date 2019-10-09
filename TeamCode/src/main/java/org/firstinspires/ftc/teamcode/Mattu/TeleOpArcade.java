package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Arcade;

@TeleOp(name = "Arcade")
public class TeleOpArcade extends OpMode {

    Arcade driveSystem;

    public void init() {
        driveSystem = new Arcade(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }

    public void loop() {
        driveSystem.drive(gamepad1);
    }
}
