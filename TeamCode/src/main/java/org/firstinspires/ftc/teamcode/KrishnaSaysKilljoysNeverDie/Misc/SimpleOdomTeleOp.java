package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;

@TeleOp(name="simple odom")
public class SimpleOdomTeleOp extends OpMode {

    //used for encoders
    private static final double     COUNTS_PER_MOTOR_REV    = 360; //360 CPR
    private static final double     WHEEL_DIAMETER_INCHES   = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV /(WHEEL_DIAMETER_INCHES * Math.PI);

    //Drive train
    Headless driveSystem;


    DcMotor fRight, fLeft, bRight, bLeft, encX;


    boolean startAccel;

    public void init() {
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        encX = hardwareMap.dcMotor.get("encX");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        fRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize drive train
        driveSystem = new Headless(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight);
        encX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void loop() {
        startAccel = gamepad1.x;

        if (startAccel) {
            driveSystem.setAccel();
        }

        driveSystem.drive(gamepad1);

        telemetry.addData("Encoder x: ", encX.getCurrentPosition());
    }

}
