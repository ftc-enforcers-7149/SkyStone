/*
package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;


//@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {

    //Intake
    IntakeVer0 intake;
    //Drive train
    Headless driveSystem;

    public void init() {

        intake = new IntakeVer0(hardwareMap, "intakeL", "intakeR", true, false);

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }

    public void loop() {

        //Drive
        driveSystem.drive(gamepad1);

        intake.runIntake(gamepad1);
    }


}
*/
