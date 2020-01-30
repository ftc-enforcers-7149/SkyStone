package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV3;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

@Autonomous(name = "OdomToPoint")
public class OdomToPoint extends OpMode {

    DriveTrainV3 driveTrain;

    OdometryPosition.Direction direction;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    int step = 0;


    public void init() {

        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");

        //Motor directions
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void start() {
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveTrain = new DriveTrainV3(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
    }

    public void loop() {
        driveTrain.updateOdom(direction);
        telemetry.addData("Position: ", "(" + driveTrain.getPosX() + ", " + driveTrain.getPosY() + ")");
        telemetry.addData("Raw X and Y: ", "(" + driveTrain.getRawX() + ", " + driveTrain.getRawY() + ")");
        telemetry.addData("Heading: ", driveTrain.getHeading());
        telemetry.addLine();

        switch (step) {
            case 0:
                //This is the right format for driving to a point
                //It will keep driving until at the point, then it will stop and move on
                if (driveTrain.driveToPoint(0, 10, 0.5)) {
                    direction = OdometryPosition.Direction.TURNING;
                    step++;
                }
                break;
            case 1:
                if (driveTrain.rotate(90,180)) {
                    direction = OdometryPosition.Direction.FORWARD;
                    step++;
                }
                break;
            case 2:
                if (driveTrain.driveToPoint(0, 20, 0.5)) {
                    step++;
                }
                break;
        }
    }

    public void stop() {
        fRight.setPower(0);
        bRight.setPower(0);
        fLeft.setPower(0);
        bLeft.setPower(0);
    }
}


