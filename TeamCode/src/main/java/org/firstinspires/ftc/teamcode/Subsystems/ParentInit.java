package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * class used as the init that all files can inherit
 */
public class ParentInit extends OpMode {
    protected Servo lArm, rArm, lGrab, rGrab;
    protected Servo fLFound, fRFound, bLFound, bRFound;
    protected DcMotor fRight,fLeft,bRight,bLeft,lift;

    protected Webcam webcam;
    protected DriveTrainV1 driveTrainV1;
    protected FoundationV2 foundation;
    protected Claw claw;
    protected Gyroscope gyro;

    //Distance Sensors
    protected DistanceSensor distanceL, distanceR, distanceC;
    protected ColorSensor color;

    private boolean initialize = true;

    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        distanceC = hardwareMap.get(DistanceSensor.class, "distanceC");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");

        color = hardwareMap.get(ColorSensor.class, "color");

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lArm.setPosition(0.95);
        rArm.setPosition(0.81);

        fLFound.setPosition(1);
        bLFound.setPosition(1);

        fRFound.setPosition(1);
        bRFound.setPosition(1);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("sensor:",distanceC.getDeviceName());
    }

    public void init_loop() {
        if (initialize) {
            webcam=new Webcam(hardwareMap);
            foundation =new FoundationV2(fLFound,fRFound,bLFound,bRFound);
            claw=new Claw(lArm,rArm,lGrab,rGrab);
            initialize = false;
        }
    }

    public void start(){
        driveTrainV1 =new DriveTrainV1(telemetry,fLeft,fRight,bLeft,bRight,gyro);
    }

    public void loop(){}

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }



}
