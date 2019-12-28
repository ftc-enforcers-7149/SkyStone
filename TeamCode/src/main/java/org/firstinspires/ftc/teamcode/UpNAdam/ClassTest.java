package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV2;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import java.util.Locale;

@Autonomous(name="auto class")
public class ClassTest extends OpMode {

    public DcMotor fLeft, fRight, bLeft, bRight;
    Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    int step=0;
    Gyroscope gyro;
    DriveTrainV2 driveTrain;

    DistanceSensor distanceL, distanceR, distanceC;
    ColorSensor color;

    BNO055IMU imu;
    Orientation angles;

    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Color sensor
        color = hardwareMap.colorSensor.get("color");

        //Distance sensors
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start(){
        gyro=new Gyroscope(telemetry,hardwareMap);
        driveTrain = new DriveTrainV2(telemetry,fLeft,fRight,bLeft,bRight,gyro);
    }

    public void loop() {
        telemetry.addData("Degree: ",gyro.getRawYaw());

        switch (step) {
            case 0:
                driveTrain.setTime(1000);
                step++;
                break;
            case 1:
                if (driveTrain.delay()) {
                    driveTrain.setDist(24);
                    step++;
                }
                break;
            case 2:
                if (driveTrain.driveStraight(Directions.FORWARD, 0.2)) {
                    step++;
                }
                break;
            case 3:
                telemetry.addLine("Finished");
                break;
        }

        telemetry.addData("Step: ", step);
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
