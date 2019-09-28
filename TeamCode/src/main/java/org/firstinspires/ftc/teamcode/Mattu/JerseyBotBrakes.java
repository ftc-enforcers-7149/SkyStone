package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "JerseyBrakes")
public class JerseyBotBrakes extends OpMode {

    DcMotor mLeft, mRight;

    public void init() {
        mLeft = hardwareMap.dcMotor.get("mLeft");
        mRight = hardwareMap.dcMotor.get("mRight");

        mLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mRight.setDirection(DcMotorSimple.Direction.REVERSE);

        mLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        mLeft.setPower(gamepad1.left_stick_y);
        mRight.setPower(gamepad1.right_stick_y);
    }

    public void stop() {
        mLeft.setPower(0);
        mRight.setPower(0);
    }
}
