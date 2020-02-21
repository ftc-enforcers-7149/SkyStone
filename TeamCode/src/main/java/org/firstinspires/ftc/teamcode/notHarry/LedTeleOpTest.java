package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@TeleOp(name = "ledTeleOpTest")
public class LedTeleOpTest extends OpMode {
    //Drive train
    Headless driveSystem;

    //Hardware
    Servo fLFound, fRFound, bLFound, bRFound;
    Servo lArm, rArm, lGrab, rGrab;
    DcMotor fRight, fLeft, bRight, bLeft, liftMotor;
    DistanceSensor distanceLift;

    Claw claw;
    FoundationV2 foundation;
    ledLiftTest ledLift;

    float armUp;
    boolean isBreak = false;
    float liftUp, liftDown;
    boolean lFoundationDown, rFoundationDown;
    float grab;
    boolean startAccel;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    public void init() {


        distanceLift = hardwareMap.get(DistanceSensor.class, "distanceLift");


        liftMotor = hardwareMap.dcMotor.get("lift");

        //Led Blinkin driver
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        //Lift brake
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        foundation = new FoundationV2(fLFound, fRFound, bLFound, bRFound);
        claw = new Claw(lArm, rArm, lGrab, rGrab);
        ledLift = new ledLiftTest(liftMotor, distanceLift);
        blinkinLedDriver.setPattern(pattern);
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    }

    public void loop() {

        blinkinLedDriver.setPattern(pattern);


        //Inputs

        liftUp = gamepad1.right_trigger;
        liftDown = gamepad1.left_trigger;
        lFoundationDown = gamepad1.left_bumper || gamepad2.x;
        rFoundationDown = gamepad1.right_bumper || gamepad2.b;
        startAccel = gamepad1.x;


        //Drive
        driveSystem.drive(gamepad1);

        //LEDs
        if (distanceLift.getDistance(DistanceUnit.CM) <= 3) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        }





        //Lift
        //ledLift.liftSet(gamepad1);



        //telemetry.addData("Lift Level", ledLift.getLevel());

    }

    public void stop(){
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
        liftMotor.setPower(0);
    }
}
