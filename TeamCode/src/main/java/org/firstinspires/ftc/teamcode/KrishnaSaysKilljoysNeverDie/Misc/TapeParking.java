package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeParking extends OpMode {

    Servo turret, shoot;
    boolean turretLeft, turretRight, shootOut, shootIn;

    public void init() {

        turret = hardwareMap.servo.get("turret");
        shoot = hardwareMap.servo.get("shoot");

    }

    public void loop() {

    }

    public void stop() {}

}
