package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "OdomToPoint")
public class OdomToPoint extends OpMode {

    //Odometry calculations
    OdometryPosition oP;
    OdometryPosition.Direction direction = OdometryPosition.Direction.FORWARD;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    int state=0;


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
    }

    public void start() {
        oP = new OdometryPosition(hardwareMap, "encX", "encY", 0, 0, gyroscope);
        oP.reverseY();
    }

    public void loop() {
        oP.updatePosition(direction);
        telemetry.addData("Encoder Y: ", oP.positionY);
        telemetry.addData("Encoder X: ", oP.positionX);
        telemetry.addData("Heading: ", oP.getHeading());

        switch (state) {
            case 0:
                if (!driveToPoint(3, 10, 0.6)) {
                    fLeft.setPower(0);
                    fRight.setPower(0);
                    bLeft.setPower(0);
                    bRight.setPower(0);
                    state++;
                }
                break;
            case 1:
                if (!driveInRef(-3, -5, 0.6)) {
                    fLeft.setPower(0);
                    fRight.setPower(0);
                    bLeft.setPower(0);
                    bRight.setPower(0);
                    state++;
                }
                break;
        }
    }

    public boolean driveInRef(double xDist, double yDist, double lim) {
        return driveToPoint(oP.positionX + xDist, oP.positionY + yDist, lim);
    }

    public boolean driveToPoint(double x, double y, double lim) {
        double relativeX = x - oP.positionX;
        double relativeY = y - oP.positionY;

        double r = Math.hypot(relativeX, relativeY);
        double robotAngle = Math.atan2(relativeY, relativeX) - Math.toRadians(cvtDegrees(oP.getHeading())) + Math.PI / 4;

        double v1 = r * Math.sin(robotAngle);
        double v2 = r * Math.cos(robotAngle);
        double v3 = r * Math.cos(robotAngle);
        double v4 = r * Math.sin(robotAngle);

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(v1), Math.abs(v2)), Math.max(Math.abs(v3), Math.abs(v4)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            v1 /= max * (1 / 0.9);
            v2 /= max * (1 / lim);
            v3 /= max * (1 / lim);
            v4 /= max * (1 / lim);
        }

        if (Math.abs(relativeX) > Math.abs(relativeY)) {
            if (oP.positionX < x) {
                fLeft.setPower(v1);
                fRight.setPower(v2);
                bLeft.setPower(v3);
                bRight.setPower(v4);
            }
            else {
                fLeft.setPower(v1);
                fRight.setPower(v2);
                bLeft.setPower(v3);
                bRight.setPower(v4);
            }
        }
        else {
            if (oP.positionY < y) {
                fLeft.setPower(v1);
                fRight.setPower(v2);
                bLeft.setPower(v3);
                bRight.setPower(v4);
            }
            else {
                fLeft.setPower(v1);
                fRight.setPower(v2);
                bLeft.setPower(v3);
                bRight.setPower(v4);
            }
        }

        if (Math.abs(relativeX) < 2 && Math.abs(relativeY) < 2) {
            return false;
        }

        return true;
    }

    private double cvtDegrees(double heading) {
        if (heading >= 0 && heading < 90) {
            return -heading + 90;
        }
        return -heading + 450;
    }

    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}


