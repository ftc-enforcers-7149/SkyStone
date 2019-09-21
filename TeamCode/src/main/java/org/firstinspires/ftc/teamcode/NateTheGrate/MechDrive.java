package org.firstinspires.ftc.teamcode.NateTheGrate;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MechDrive")
public class MechDrive extends OpMode {


    DcMotor fRight, fLeft, bLeft, bRight;

    float lTrig, rTrig;

    double rightY, leftY;



    public void init() {
        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);
        bLeft.setDirection(DcMotor.Direction.REVERSE);

    }


    public void loop() {
        lTrig = gamepad1.left_trigger;
        rTrig = gamepad1.right_trigger;
        rightY = gamepad1.right_stick_y;
        leftY = gamepad1.left_stick_y;

        if (lTrig > 0.1) {
            fRight.setPower(-lTrig / 1.2);
            fLeft.setPower(lTrig / 1.2);
            bLeft.setPower(-lTrig / 1.2);
            bRight.setPower(lTrig / 1.2);
        }
        else if (rTrig > 0.1) {
            fRight.setPower(rTrig / 1.2);
            fLeft.setPower(-rTrig / 1.2);
            bLeft.setPower(rTrig / 1.2);
            bRight.setPower(-rTrig / 1.2);
        }
        else {
            fRight.setPower(rightY);
            fLeft.setPower(leftY);
            bLeft.setPower(leftY);
            bRight.setPower(rightY);
        }
    }




    public void stop (){

    }

}
