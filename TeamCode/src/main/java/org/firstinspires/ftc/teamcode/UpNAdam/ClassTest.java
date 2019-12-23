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

    }
public void start(){
    gyro=new Gyroscope(telemetry,hardwareMap);
    driveTrain =new DriveTrainV2(telemetry,fLeft,fRight,bLeft,bRight,gyro);

}
    public void loop() {
        telemetry.addData("degree",gyro.getRawYaw());

        switch (step) {
            case 0:
                if (driveTrain.driveRange(distanceL, 20, "left")) {
                    step++;
                }
                break;
        }
    }

    public void stop() {
    }


}
