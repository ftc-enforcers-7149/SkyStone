package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

@TeleOp(name="Custom Encoder Testing")
public class CustomEncoderTest extends OpMode {
    //Drive train
    Headless driveSystem;
    Gyroscope gyroscope;
    DcMotor fRight, fLeft, bRight, bLeft;

    //Odometry
    CustomOdometryPosition odometry;
    CustomOdometryPosition.Direction dir = CustomOdometryPosition.Direction.FORWARD;

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
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);

        //Odometry setup
        odometry = new CustomOdometryPosition(hardwareMap, 0, 0, gyroscope);
        odometry.reverseY();

        odometry.startThread();
    }

    public void loop() {
        odometry.updatePosition(dir);
        driveSystem.drive(gamepad1);

        if (gamepad1.a) {
            dir = CustomOdometryPosition.Direction.TURNING;
        } else if (gamepad1.b) {
            dir = CustomOdometryPosition.Direction.FORWARD;

            odometry.resetX();
            odometry.resetY();
        }

        telemetry.addData("posX:", odometry.getPositionX());
        telemetry.addData("posY:", odometry.getPositionY());
        telemetry.addData("heading:", odometry.getHeading());
    }

    public void stop() {
        odometry.stopThread();

        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}
