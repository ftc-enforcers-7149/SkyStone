package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "Headless")
public class HeadlessMode extends OpMode {

    //DC Motors
    DcMotor fLeft, fRight, bRight, bLeft;

    //IMU variables
    BNO055IMU imu;
    Orientation angles;
    double angle, offset;

    //Variables for inputs
    double joyX, joyY, turnX;
    boolean resetAngle;

    public void init() {
        //Hardware mapping the motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

        //Reversing left motors
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            }
        });

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void loop() {
        //Getting inputs
        angle = angles.firstAngle;
        joyY = -gamepad1.left_stick_y;
        joyX = -gamepad1.left_stick_x;
        turnX = gamepad1.right_stick_x;
        resetAngle = gamepad1.y;

        //Only set brakes if no inputs are given from the joysticks
        //this makes it easier for the robot to move diagonally
        if (joyY == 0 && joyX == 0 && turnX == 0) {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else {
            bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //Offset the angle if the imu is incorrect
        if (resetAngle) {
            offset = angle;
        }

        //Telemetry for angles
        telemetry.addData("Angle: ", angle - offset);
        telemetry.addData("Converted angle: ", cvtDegrees(angle - offset));

        //r is used to scale the power of the motors depending on how much the joysticks are pushed
        //robotAngle is the directional angle (radians) that the robot wants to go in terms of itself.
        //45 degress is adding in radians because that is the small angle on a right triangle.
        //v1 - v4 are the velocities for each motor. r is multiplied here.
        //Sine and cosine are applied to their respective diagonal's wheels.
        //turnX is added to and subtracted from their respective side's wheels.
        double r = Math.hypot(joyX, joyY);
        double robotAngle = Math.atan2(joyY, joyX) - Math.toRadians(cvtDegrees(angle - offset)) + Math.PI / 4;
        double v1 = r * Math.sin(robotAngle) + turnX;
        double v2 = r * Math.cos(robotAngle) - turnX;
        double v3 = r * Math.cos(robotAngle) + turnX;
        double v4 = r * Math.sin(robotAngle) - turnX;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above 0.8 power by scaling them all down if such a thing might occur.
        if (max > 0.8) {
            v1 /= max*1.25;
            v2 /= max*1.25;
            v3 /= max*1.25;
            v4 /= max*1.25;
        }

        //Telemetry for the motor velocities
        telemetry.addData("fLeft: ", v1);
        telemetry.addData("fRight: ", v2);
        telemetry.addData("bLeft: ", v3);
        telemetry.addData("bRight: ", v4);

        //Setting each velocity to its respective motor
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }

    //This converts degrees to work with sine and cosine.
    //The equations were made in Desmos by plotting certain points (input, output)
    //Equation 1: y = -x + 90
    //Equation 2: y = -x + 450
    public double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }
}