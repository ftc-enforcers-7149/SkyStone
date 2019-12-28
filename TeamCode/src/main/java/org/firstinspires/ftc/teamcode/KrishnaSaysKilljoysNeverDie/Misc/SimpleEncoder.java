package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryEncoder;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@TeleOp(name = "Simple Encoder")
public class SimpleEncoder extends OpMode {

    OdometryPosition oP;
    Gyroscope gyroscope;

    public void init(){}

    public void start() {
        oP = new OdometryPosition(hardwareMap, "encX", "encY", 0, 0, gyroscope);
    }

    public void loop() {
        oP.updatePosition(OdometryPosition.Direction.FORWARD);
        telemetry.addData("encoder x: ", oP.positionX);
    }

}
