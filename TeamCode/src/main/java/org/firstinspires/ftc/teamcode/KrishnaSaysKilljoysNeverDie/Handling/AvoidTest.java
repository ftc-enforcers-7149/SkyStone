package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Handling;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;

@TeleOp(name = "avoid stuff boyo")
public class AvoidTest extends OpMode {


    //Devices
    DcMotor fL, fR, bL, bR;
    MovementDetectionClass movement;




    public void init() {

        movement = new MovementDetectionClass(hardwareMap, "distC", "distL", "distR", fL, fR, bL, bR);

    }

    public void loop() {

        movement.update();

        if(gamepad1.a) {
            Log.d("User input?: ", "Yes");
        }

        if(movement.isFrontClose()) {
            Log.d("Front close: ", "True");
            telemetry.addLine("Front close");
        }

        if(movement.isLeftClose()) {
            Log.d("Left close: ", "True");
            telemetry.addLine("Left Close");
        }

        if(movement.isRightClose()) {
            Log.d("Right close: ", "True");
            telemetry.addLine("Right Close");
        }

        if(movement.isFrontMoving()) {
            Log.d("Front moving: ", "True");
            telemetry.addLine("Front Moving");
        }

    }

    public void stop() {

    }





}
