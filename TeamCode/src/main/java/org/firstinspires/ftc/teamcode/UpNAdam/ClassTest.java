package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV2;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@Autonomous(name="auto class")
public class ClassTest extends OpMode {

    public DcMotor fLeft, fRight, bLeft, bRight;
    Gyroscope gyro;
    DriveTrainV2 driveTrain;

    /*DistanceSensor distanceL, distanceR, distanceC;
    ColorSensor color;*/

    int step=0;

    public void init() {
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        /*//Color sensor
        color = hardwareMap.colorSensor.get("color");

        //Distance sensors
        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");*/

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
        gyro = new Gyroscope(telemetry,hardwareMap);
        driveTrain = new DriveTrainV2(telemetry,fLeft,fRight,bLeft,bRight,gyro);
    }

    public void loop() {
        switch (step) {
            case 0:
                if (driveTrain.delay(100)) {
                    step++;
                }
                break;
            case 1:
                if (driveTrain.driveStraight(Directions.FORWARD, 24, 0.3)) {
                    step++;
                }
                break;
            case 2:
                if (driveTrain.rotate(270)) {
                    step++;
                }
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
