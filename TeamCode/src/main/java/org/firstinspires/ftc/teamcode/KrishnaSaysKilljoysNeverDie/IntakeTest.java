package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {

    IntakeVer0 intake;

    public void init() {
        intake = new IntakeVer0(hardwareMap, "intakeL", "intakeR", true, false);
    }

    public void loop() {
        intake.runIntake(gamepad1);
    }

}
