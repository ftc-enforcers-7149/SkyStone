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

@TeleOp(name = "MecanumDrives")
public class MecanumDrives extends OpMode {

    //DC Motors and velocity variables
    DcMotor fLeft, fRight, bLeft, bRight;
    double v1, v2, v3, v4; //In same order as motors

    //IMU variables
    BNO055IMU imu;
    Orientation angles;
    double angle, offset;

    //Variables for inputs
    double leftX, leftY, rightX, rightY;
    double leftT, rightT;
    boolean resetAngle, changeMode;

    //Variable for mode
    //0 is tank
    //1 is arcade
    //2 is headless
    int mode;

    //Power limit
    double lim;

    public void init() {
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

        //Initialize variables
        mode = 0;
        lim = 0.9;
        v1 = 0;
        v2 = 0;
        v3 = 0;
        v4 = 0;
    }

    public void loop() {
        //Getting inputs
        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;
        changeMode = gamepad1.right_stick_button;


        //Tank Drive
        if (mode == 0) {
            //Specific inputs
            rightY = gamepad1.right_stick_y;
            leftT = gamepad1.left_trigger;
            rightT = gamepad1.right_trigger;

            v1 = leftY;
            v2 = rightY;
            v3 = leftY;
            v4 = rightY;

            if (rightT > 0.1) {
                v1 = rightT;
                v2 = -rightT;
                v3 = -rightT;
                v4 = rightT;
            }
            else if (leftT > 0.1) {
                v1 = -rightT;
                v2 = rightT;
                v3 = rightT;
                v4 = -rightT;
            }

            if (changeMode) {
                mode = 1;
            }
        }


        //Arcade Drive
        else if (mode == 1) {
            v1 = leftY + leftX + rightX;
            v2 = leftY - leftX - rightX;
            v3 = leftY - leftX + rightX;
            v4 = leftY + leftX - rightX;

            if (changeMode) {
                mode = 2;
            }
        }


        //Headless Arcade Drive
        else if (mode == 2) {
            //Specific inputs
            angle = angles.firstAngle;
            resetAngle = gamepad1.y;

            //Fixing inputs
            leftY = -leftY;
            leftX = -leftX;

            //Only set brakes if no inputs are given from the joysticks
            //this makes it easier for the robot to move diagonally
            if (leftY == 0 && leftX == 0 && rightX == 0) {
                bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
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
            //45 degrees is adding in radians because that is the small angle on a right triangle.
            //v1 - v4 are the velocities for each motor. r is multiplied here.
            //Sine and cosine are applied to their respective diagonal's wheels.
            //rightX is added to and subtracted from their respective side's wheels.
            double r = Math.hypot(leftX, leftY);
            double robotAngle = Math.atan2(leftY, leftX) - Math.toRadians(cvtDegrees(angle - offset)) + Math.PI / 4;
            v1 = r * Math.sin(robotAngle) + rightX;
            v2 = r * Math.cos(robotAngle) - rightX;
            v3 = r * Math.cos(robotAngle) + rightX;
            v4 = r * Math.sin(robotAngle) - rightX;

            if (changeMode) {
                mode = 0;
            }
        }

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1/lim);
            v2 /= max * (1/lim);
            v3 /= max * (1/lim);
            v4 /= max * (1/lim);
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