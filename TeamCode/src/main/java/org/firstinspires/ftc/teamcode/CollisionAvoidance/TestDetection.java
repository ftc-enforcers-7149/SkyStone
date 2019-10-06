package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

import java.util.Arrays;

@TeleOp(name = "Detect Movement")
public class TestDetection extends OpMode {
    //Instance classes for detection and movement
    DetectionClass detection;
    DriveTrain driveTrain;

    //Hardware
    private DcMotor fLeft, fRight, bLeft, bRight;
    BNO055IMU imu;
    private Orientation angles;

    double leftY, leftX, rightX;
    double v1, v2, v3, v4;
    double prevTime;
    double lim;
    boolean isMoving;

    public void init() {
        detection = new DetectionClass(hardwareMap, "distanceL", "distanceR", "distanceC");

        //Hardware mapping the motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

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

        driveTrain = new DriveTrain(fLeft, fRight, bLeft, bRight, angles);

        //Initialize variables
        lim = 0.9;
        isMoving = false;
    }

    public void start() {
        prevTime = System.currentTimeMillis();
    }

    public void loop() {
        //Getting inputs
        leftY = -gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = -gamepad1.right_stick_x;

        detection.getInput();

        if (System.currentTimeMillis() - prevTime > 100) {
            isMoving = detection.isObjectMoving(fLeft);

            prevTime += 100;
        }

        telemetry.addData("Moving Obstacle? ", isMoving);
        telemetry.addData("Sensor State (left, front, right)", Arrays.toString(detection.getSensorState()));
        telemetry.addLine(detection.getMovingData());

        v1 = leftY + leftX + rightX;
        v2 = leftY - leftX - rightX;
        v3 = leftY - leftX + rightX;
        v4 = leftY + leftX - rightX;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1/lim);
            v2 /= max * (1/lim);
            v3 /= max * (1/lim);
            v4 /= max * (1/lim);
        }

        //Setting each velocity to its respective motor
        fLeft.setPower(v1);
        fRight.setPower(v2);
        bLeft.setPower(v3);
        bRight.setPower(v4);
    }
}