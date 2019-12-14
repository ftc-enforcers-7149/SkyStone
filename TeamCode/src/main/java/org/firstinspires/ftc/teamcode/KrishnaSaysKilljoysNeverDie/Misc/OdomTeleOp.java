package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@TeleOp(name = "OdomTeleOp")
public class OdomTeleOp extends OpMode {

    //Drive train
    Headless driveSystem;

    //O do met ry
    OdometryPosition oP;

    DcMotor fRight, fLeft, bRight, bLeft;


    boolean startAccel;


    public void init() {

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight);

    }


    public void start() {
        oP = new OdometryPosition(hardwareMap, "encX", "encY", "imu", 0, 0);
        oP.reverseX();
    }

    public void loop() {
        startAccel = gamepad1.x;

        OdometryPosition.Direction direction;

        if(gamepad1.y) {
            direction = OdometryPosition.Direction.TURNING;
        }
        else {
            direction = OdometryPosition.Direction.FORWARD;
        }

        if (startAccel) {
            driveSystem.setAccel();
        }

        driveSystem.drive(gamepad1);

        oP.updatePosition(direction);
        telemetry.addData("encoder x: ", oP.positionX);
        telemetry.addData("heading: ", oP.getHeading());

        telemetry.addLine("WATCH STAR WARS TROS IN THEATERS DEC 20");

    }

    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }

}