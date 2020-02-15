package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;


@TeleOp(name = "demoBot")
public class demoBot extends OpMode {
    Headless driveSystem;
    Gyroscope gyro;

    DcMotor fLeft, fRight, bLeft, bRight;
    public void init(){
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Drive System initialization
        gyro = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyro, fLeft, fRight, bLeft, bRight, false);//false);

    }
    public void loop(){
        driveSystem.drive(gamepad1);

    }
    public void stop(){
        driveSystem.stop();
    }

}
