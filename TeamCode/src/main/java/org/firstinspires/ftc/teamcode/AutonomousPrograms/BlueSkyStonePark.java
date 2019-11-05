package org.firstinspires.ftc.teamcode.AutonomousPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Webcam;

@Autonomous(name="Blue SkyStone Park")
public class BlueSkyStonePark extends OpMode {
    public Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    int step=0;

    Webcam webcam;
    DriveTrain driveTrain;

    String position="";
    public void init(){
        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        //Servos up
        rFound.setPosition(0);
        lFound.setPosition(0);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void start(){
        webcam=new Webcam(hardwareMap);
        driveTrain=new DriveTrain(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
    }
    public void loop(){
        switch(step){
            case 0:driveTrain.driveStraight("forward",14);
                break;
            case 1:position=webcam.getPosition();
                break;
        }
        step++;
        telemetry.addData("position",position);
    }
    public void stop(){

    }
}
