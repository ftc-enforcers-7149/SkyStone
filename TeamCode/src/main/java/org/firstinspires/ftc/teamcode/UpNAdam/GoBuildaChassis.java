package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
@TeleOp(name="GoBuilda")
public class GoBuildaChassis extends OpMode {


    //Drive train
    Headless driveSystem;

    public void init(){
        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, "fLeft", "fRight", "bLeft", "bRight");
    }
    public void loop(){
        //Drive
        driveSystem.drive(gamepad1);
    }
}
