package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CollisionData extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //Distance Sensors
    DistanceSensor front, distBLeft, distBRight;


    public void init() {
        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        front = hardwareMap.get(DistanceSensor.class, "front");
        distBLeft = hardwareMap.get(DistanceSensor.class, "distBLeft");
        distBRight = hardwareMap.get(DistanceSensor.class, "distBRight");


    }

    public void loop() {
        //Drive
        fRight.setPower(gamepad1.right_stick_y);
        fLeft.setPower(gamepad1.left_stick_y);
        bLeft.setPower(gamepad1.left_stick_y);
        bRight.setPower(gamepad1.right_stick_y);


        //Variables
        double fRightPower = fRight.getPower();
        double fDist = front.getDistance(DistanceUnit.CM);
        double bLDist = distBLeft.getDistance(DistanceUnit.CM);
        double bRDist = distBRight.getDistance(DistanceUnit.CM);


        //Telemetry
        telemetry.addData("Motor speed: ", fRight.getPower());
        telemetry.addData("Front Dist: ", fDist);
        telemetry.addData("Back L Dist: ", bLDist);
        telemetry.addData("Back R Dist: ", bRDist);





    }

    public void stop() {

    }

}
