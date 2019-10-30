package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Handling;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;

public class AvoidTest extends OpMode {


    //Devices
    DcMotor fL, fR, bL, bR;
    DistanceSensor distC, distL, distR;
    MovementDetectionClass movement;




    public void init() {

        movement = new MovementDetectionClass(distC, distL, distR, fL, fR, bL, bR);

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

        if(movement.isLeftMoving()) {
            Log.d("Left moving: ", "True");
            telemetry.addLine("Left Moving");
        }

        if(movement.isRightMoving()) {
            Log.d("Right moving: ", "True");
            telemetry.addLine("Right Moving");
        }


    }

    public void stop() {

    }





}
