package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Arcade;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@TeleOp(name = "OdomTeleOp")
public class OdomTeleOp extends OpMode {

    //Drive train
    Headless driveSystem;

    //O do met ry
    OdometryPosition oP;

    DcMotor fRight, fLeft, bRight, bLeft;

    Gyroscope gyroscope;


    boolean startAccel;

    //used for encoders (y)
    private static final double     COUNTS_PER_MOTOR_REVY    = 400;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESY  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHY        = COUNTS_PER_MOTOR_REVY /(WHEEL_DIAMETER_INCHESY * Math.PI);

    //used for encoders (x)
    private static final double     COUNTS_PER_MOTOR_REVX    = 400;  //1440 for 1 enc //512 for another(x) 400 for (y) //
    private static final double     WHEEL_DIAMETER_INCHESX  = 1.49606299d ;     // For figuring circumference
    public static final double     COUNTS_PER_INCHX      = COUNTS_PER_MOTOR_REVX /(WHEEL_DIAMETER_INCHESX * Math.PI);


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

        gyroscope = new Gyroscope(telemetry, hardwareMap);
        //Initialize drive train
        driveSystem = new Headless(gyroscope, fLeft, fRight, bLeft, bRight);


    }

    public void start() {
        oP = new OdometryPosition(hardwareMap, "encX", "encY", 0, 0, gyroscope);
        oP.reverseY();
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

        driveSystem.drive(gamepad1);

        oP.updatePosition(direction);
        telemetry.addData("encoder x: ", oP.positionX);
        telemetry.addData("encoder y:",oP.positionY);
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