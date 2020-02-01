package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2_1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV3;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name = "Red Quarry Odom Auto v2")
public class RedQuarryOdometryAuto extends ParentInit {

    DriveTrainV3 driveTrain;

    OdometryPosition.Direction direction;

    //Motors and imu
    DcMotor fRight, fLeft, bRight, bLeft;
    Gyroscope gyroscope;

    String position="";

    int step = 0;

    public void init() {

        super.init();

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

        direction = OdometryPosition.Direction.FORWARD;
    }

    public void init_loop() {
        super.init_loop();

        switch(step){
            case 0:
                position=webcam.getQueuePos(telemetry);
                step++;
                break;
        }
        telemetry.addData("position",position);
    }

    public void start() {
        gyroscope = new Gyroscope(telemetry, hardwareMap);
        driveTrain = new DriveTrainV3(hardwareMap, telemetry, fLeft, fRight, bLeft, bRight, gyroscope);
        driveTrain.setX(31);
        claw.down();
    }

    public void loop() {
        driveTrain.updateOdom(direction);
        telemetry.addData("Position: ", "(" + driveTrain.getPosX() + ", " + driveTrain.getPosY() + ")");
        telemetry.addData("Heading: ", driveTrain.getHeading());
        telemetry.addLine();

        switch (step) {
            case 1:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(36, 22, 0.5)) {
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(28, 22, 0.5)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(20,22, 0.5)) {
                        step++;
                    }
                }
                break;
            case 2:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(37, 31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(28, 31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(20,31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                break;
            case 3:
                claw.halfUp();
                step++;
                break;
            case 4:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(37, 24, 0.5)) {
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(28, 24, 0.5)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(20,24, 0.5)) {
                        step++;
                    }
                }
                break;
            case 5:
                if (driveTrain.driveToPoint(108, 24, 0.5)) {
                    claw.down();
                    claw.release();
                    step++;
                }
                break;
            case 6:
                claw.halfUp();
                step++;
                break;
            case 7:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(12, 24, 0.5)) {
                        claw.down();
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(4, 24, 0.5)) {
                        claw.down();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(0,24, 0.5)) {
                        claw.down();
                        step++;
                    }
                }
                break;
            case 8:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(12, 31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(4, 31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(0,31, 0.5)) {
                        claw.grab();
                        step++;
                    }
                }
                break;
            case 9:
                claw.halfUp();
                step++;
                break;
            case 10:
                if (position.equals("right")) {
                    if (driveTrain.driveToPoint(12, 24, 0.5)) {
                        step++;
                    }
                }
                else if (position.equals("center")) {
                    if (driveTrain.driveToPoint(4, 24, 0.5)) {
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveToPoint(0,24, 0.5)) {
                        step++;
                    }
                }
                break;
            case 11:
                if (driveTrain.driveToPoint(96, 24, 0.5)) {
                    claw.down();
                    claw.release();
                    step++;
                }
                break;
            case 12:
                claw.halfUp();
                step++;
                break;
            case 13:
                if (driveTrain.driveToPoint(63, 24, 0.5)) {
                    step++;
                    requestOpModeStop();
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
