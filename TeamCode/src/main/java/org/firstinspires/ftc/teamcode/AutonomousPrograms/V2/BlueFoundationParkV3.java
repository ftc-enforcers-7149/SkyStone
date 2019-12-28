package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV2;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@Autonomous(name = "Blue Foundation ParkV3")
//@Disabled                            // Comment this out to add to the opmode list
public class BlueFoundationParkV3 extends OpMode {

    public Servo lArm, rArm, lGrab, rGrab;
    public Servo fLFound, fRFound, bLFound, bRFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    DriveTrainV2 driveTrain;
    Gyroscope gyro;

    FoundationV2 foundation;
    Claw claw;
    ColorSensor color;

    int step=0;

    public void init() {

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

        //Color sensor
        color = hardwareMap.colorSensor.get("color");

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

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lArm.setPosition(0.95);
        rArm.setPosition(0.81);

        fLFound.setPosition(1);
        bLFound.setPosition(1);

        fRFound.setPosition(1);
        bRFound.setPosition(1);

    }

    public void start(){
        gyro = new Gyroscope(telemetry,hardwareMap);
        driveTrain = new DriveTrainV2(telemetry,fLeft,fRight,bLeft,bRight,gyro);
        foundation =new FoundationV2(fLFound,fRFound,bLFound,bRFound);
        claw=new Claw(lArm,rArm,lGrab,rGrab);
    }

    // Loop and update the dashboard
    public void loop() {
        switch(step){
            //Move to and grab foundation
            case 0:
                driveTrain.setDist(47);
                break;
            case 1:
                if (driveTrain.driveStraight(Directions.BACKWARD)) {
                    driveTrain.setTime(750);
                    step++;
                }
                break;
            case 2:
                if (driveTrain.strafeSeconds(Directions.RIGHT)) {
                    step++;
                }
                break;
            case 3:
                foundation.rDown();
                driveTrain.setTime(500);
                break;
            case 4:
                if (driveTrain.delay()) {
                    driveTrain.setTime(250);
                    step++;
                }
                break;
            case 5:
                if (driveTrain.strafeSeconds(Directions.LEFT)) {
                    step++;
                }
                break;
            //Move foundation into corner
            case 6:
                if (driveTrain.simpleTurn(45,0.45)) {
                    driveTrain.setTime(3000);
                    step++;
                }
                break;
            case 7:
                if (driveTrain.strafeSeconds(Directions.RIGHT)) {
                    driveTrain.setDist(36);
                    step++;
                }
                break;
            //Move foundation flush against wall
            case 8:
                if (driveTrain.driveStraight(Directions.FORWARD)) {
                    driveTrain.setTime(500);
                    step++;
                }
                break;
            case 9:
                if (driveTrain.strafeSeconds(Directions.LEFT)) {
                    step++;
                }
                break;
            case 10:
                foundation.rUp();
                break;
            //Navigate to line close to wall
            case 11:
                if (driveTrain.strafeToLine(color, Directions.LEFT)) {
                    step++;
                }
                break;
        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
