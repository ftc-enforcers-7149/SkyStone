package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

@Autonomous(name = "Demobot Stones")
public class DemoAutoStones extends OpMode {

    Gyroscope gyro;

    DcMotor fLeft, fRight, bLeft, bRight;

    Servo lArm, rArm, claw;

    int step = 0;

    public void init() {
        //Motor initialization
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        //lift = hardwareMap.dcMotor.get("lift");

        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servo initialization
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        claw = hardwareMap.servo.get("claw");

        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);

        gyro = new Gyroscope(telemetry, hardwareMap);

        lift(true);
        grab(false);
    }

    public void start() {
        lift(false);
    }

    public void loop() {
        telemetry.addData("Step: ", step);

        switch (step) {
            case 0:
                if (driveByTime(Directions.FORWARD, 1500, 0.7)) {
                    step++;
                }
                break;
            case 1:
                grab(true);
                step++;
                break;
            case 2:
                if (driveByTime(Directions.BACKWARD, 1000, 0.7)) {
                    lift(true);
                    step++;
                }
                break;
            case 3:
                if (driveByTime(Directions.LEFT, 1750, 0.7)) {
                    lift(false);
                    grab(false);
                    step++;
                }
                break;
            case 4:
                if (driveByTime(Directions.RIGHT, 500, 0)) {
                    lift(true);
                    step++;
                }
                break;
            case 5:
                if (driveByTime(Directions.RIGHT, 750, 0.7)) {
                    step++;
                    requestOpModeStop();
                }
        }
    }

    public void grab(boolean grab) {
        if (grab) {
            claw.setPosition(0.67);
        }
        else {
            claw.setPosition(0.95);
        }
    }

    public void lift(boolean lift) {
        if (lift) {
            lArm.setPosition(0.6);
            rArm.setPosition(0.8);
        }
        else {
            lArm.setPosition(0.15);
            rArm.setPosition(0.37);
        }
    }

    double prevTime=0, endTime=0;

    public boolean driveByTime(Directions dir, double time, double power) {
        if (time != prevTime) {
            endTime = System.currentTimeMillis() + time;
            prevTime = time;
        }

        int dirPower=1;

        if (dir == Directions.FORWARD || dir == Directions.BACKWARD) {
            if (dir == Directions.BACKWARD) {
                dirPower = -1;
            }

            if (System.currentTimeMillis() < endTime) {
                fLeft.setPower(power * dirPower);
                fRight.setPower(power * dirPower);
                bLeft.setPower(power * dirPower);
                bRight.setPower(power * dirPower);

                return false;
            }
            else {
                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);

                endTime = 0;
                prevTime = 0;
                return true;
            }
        }
        else {
            if (dir == Directions.LEFT) {
                dirPower = -1;
            }

            if (System.currentTimeMillis() < endTime) {
                fLeft.setPower(power * dirPower);
                fRight.setPower(-power * dirPower);
                bLeft.setPower(-power * dirPower);
                bRight.setPower(power * dirPower);

                return false;
            }
            else {
                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);

                endTime = 0;
                prevTime = 0;
                return true;
            }
        }
    }

    boolean last_ang=true;
    double initAngle;

    public boolean rotate(double destAngle,double speed) {
        if (last_ang) {
            initAngle=360-gyro.getYaw();
            last_ang=false;
        }

        //Get current heading
        double heading = 360-gyro.getYaw();
        double delta = gyro.getDelta(destAngle, heading);
        boolean left;
        boolean done=false;
        double mDirection;


        if(gyro.getDelta(destAngle, initAngle) > 0){
            left=false;
            mDirection=1;
        }
        else{
            left=true;
            mDirection=-1;
        }


        if(left){
            if(delta > 0){
                done=true;
            }
        }
        else{
            if(delta < 0){
                done=true;
            }
        }

        //If heading is not at destination
        if (!done) {

            telemetry.addData("Angle: ", delta);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Delta: ",delta);

            //Drive the motors so the robot turns
            fLeft.setPower(speed*mDirection);
            fRight.setPower(-speed*mDirection);
            bLeft.setPower(speed*mDirection);
            bRight.setPower(-speed*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_ang = true;

            return true;
        }

        return false;
    }
}