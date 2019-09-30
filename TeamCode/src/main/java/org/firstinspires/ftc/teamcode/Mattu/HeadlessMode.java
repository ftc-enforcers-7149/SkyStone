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
    DcMotor fLeft, fRight, bRight, bLeft; //Left and right motors

    //gyro define
    BNO055IMU imu;
    // gyro telemetry
    Orientation angles;

    double angle, offset;
    double joyX, joyY, turnX;
    boolean resetAngle;

    public void init() {
        fLeft= hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bRight = hardwareMap.dcMotor.get("bRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");

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
        angle = angles.firstAngle;
        joyY = -gamepad1.left_stick_y;
        joyX = -gamepad1.left_stick_x;
        turnX = gamepad1.right_stick_x;
        resetAngle = gamepad1.y;

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

        if (resetAngle) {
            offset = angle;
        }

        telemetry.addData("Angle: ", angle - offset);
        telemetry.addData("Converted angle: ", cvtDegrees(angle));

        double r = Math.hypot(joyX, joyY);
        double robotAngle = Math.atan2(joyY, joyX) - Math.toRadians(cvtDegrees(angle - offset)) + Math.PI / 4;
        double v1 = r * Math.sin(robotAngle) + turnX;
        double v2 = r * Math.cos(robotAngle) - turnX;
        double v3 = r * Math.cos(robotAngle) + turnX;
        double v4 = r * Math.sin(robotAngle) - turnX;

        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        if (max > 0.8) {
            v1 /= max*1.25;
            v2 /= max*1.25;
            v3 /= max*1.25;
            v4 /= max*1.25;
        }

        telemetry.addData("fLeft: ", v1);
        telemetry.addData("fRight: ", v2);
        telemetry.addData("bLeft: ", v3);
        telemetry.addData("bRight: ", v4);

        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }

    public double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }
}