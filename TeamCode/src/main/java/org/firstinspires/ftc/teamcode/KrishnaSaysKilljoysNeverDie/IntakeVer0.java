package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeVer0 {

    DcMotor intakeL, intakeR;
    Gamepad gamepad1;

    public IntakeVer0(HardwareMap hardwareMap, String left, String right) {

        intakeL = hardwareMap.dcMotor.get(left);
        intakeR = hardwareMap.dcMotor.get(right);


    }


    public void runIntake(Gamepad gamepad) {

        gamepad1 = gamepad;

        if(gamepad1.right_trigger > 0.1) { intakeR.setPower(1); }

        if(gamepad1.left_trigger > 0.1) { intakeL.setPower(1); }

        if(gamepad1.right_bumper) { intakeR.setPower(-1); }

        if(gamepad1.left_bumper) { intakeL.setPower(1); }


    }

}
