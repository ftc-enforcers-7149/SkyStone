package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV4;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;
import org.firstinspires.ftc.teamcode.Subsystems.Range;

@Autonomous(name = "Red Skystone Park V2_3")
public class RedSkyStoneParkV2_3 extends ParentInit {

    OdometryPosition.Direction direction;
    Positions position;

    int step = 0;
    double offset = 0;

    public void init() {
        super.init();
        direction = OdometryPosition.Direction.FORWARD;
    } //

    public void init_loop() { super.init_loop(); }

    public void start() {
    }

    public void loop() {
        telemetry.addData("position: ", position);
        telemetry.addData("delta: ",gyroscope.getDelta(90,gyroscope.getRawYaw()));
        telemetry.addData("heading: ",gyroscope.getYaw());
        telemetry.addData("step: ",step);

        switch (step) {
            //Senses skystone
            case 0:
                //Get skystone position
                position = webcam.getQueuePos(telemetry);
                claw.up();
                claw.grab();
                step++;
                break;
            //Moves to align with block, and sets offset
            case 1:
                if (position == Positions.RIGHT) {
                    if (driveTrain.driveStraight(Directions.FORWARD,6)) {
                        step++;
                    }
                }
                else if (position == Positions.CENTER) {
                    if (driveTrain.driveStraight(Directions.FORWARD, 0,0)) {
                        offset = 10;
                        step++;
                    }
                }
                else {
                    if (driveTrain.driveStraight(Directions.BACKWARD, 7)) {
                        offset = 16;
                        step++;
                    }
                }
                foundation.lHalf();
                break;
            //Strafes into and grabs block
            case 2:
                if (driveTrain.strafeDrive(Directions.LEFT,24, 0.6, 0)) {
                    foundation.lDown();
                    step++;
                }
                break;

            //Strafes more into block
            case 3:
                if (driveTrain.strafeDrive(Directions.LEFT,8, 0.6, 0)) {
                    step++;
                }
                break;
            //Drives forward to correct block grab
            case 4:
                if (driveTrain.driveStraight(Directions.FORWARD,2,0,0.2)) {
                    foundation.lHalf();
                    step++;
                }
                break;
            //Strafes away from block
            case 5:
                if (driveTrain.strafeDrive(Directions.RIGHT,9)) {
                    step++;
                }
                break;
            //Drives to foundation
            case 6:
                if (driveTrain.driveStraight(Directions.FORWARD,55 + offset,0)) {
                    step++;
                }
                break;
            //Strafes into foundation and releases block
            case 7:
                if (driveTrain.strafeDrive(Directions.LEFT,11,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits and puts foundation grabber up
            case 8:
                if(driveTrain.delay(500)) {
                    foundation.lUp();
                    step++;
                }
                break;
            //Strafes away from block
            case 9:
                if (driveTrain.strafeDrive(Directions.RIGHT,8,0.4)) {
                    step++;
                }
                break;
            //Drives back to second block
            case 10:
                if(position == Positions.CENTER) {
                    offset = 5;
                }
                if (driveTrain.driveStraight(Directions.BACKWARD,79 + offset,0)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits for a second
            case 11:
                if (driveTrain.delay(1000)) {
                    step++;
                }
            //Strafes to block
            case 12:
                if (driveTrain.strafeDrive(Directions.LEFT,6, 0.6, 0)) {
                    step++;
                }
                break;
            //If left, drives for 1.25 seconds. Else, sets into ready position
            case 13:
                /*if(position==Positions.LEFT) {
                    fLeft.setPower(-0.2);
                    fRight.setPower(-0.2);
                    bLeft.setPower(-0.2);
                    bRight.setPower(-0.2);
                    if (driveTrain.delay(1250)) {
                        fLeft.setPower(0);
                        fRight.setPower(0);
                        bLeft.setPower(0);
                        bRight.setPower(0);
                        step++;
                    }
                }
                else {
                    step++;
                }*/
                step++;
                break;

            case 14:
                if (driveTrain.driveStraight(Directions.FORWARD,2,0,0.2)) {
                    foundation.lHalf();
                    step++;
                }
                break;
            //Strafes away from block
            case 15:
                if (driveTrain.strafeDrive(Directions.RIGHT,12,0.4)) {
                    step++;
                }
                break;
            //Drives forward to foundation
            case 16:
                if (driveTrain.driveStraight(Directions.FORWARD,85 + offset,2,0.8)) {
                    step++;
                }
                break;
            //Strafes into foundation and grabs it
            case 17:
                if (driveTrain.strafeDrive(Directions.LEFT,14,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits for 1.25 seconds
            case 18:
                if(driveTrain.delay(750)){
                    step++;
                }
                break;
            //Turns foundation
            case 19:
                if (driveTrain.foundationTurn(90)){
                    step++;
                }
                break;
            //Strafes against wall
            case 20:
                if(driveTrain.strafeSeconds(Directions.LEFT,1500,0.7)){
                    step++;
                }
                break;
            //Drives foundation in and releases foundation
            case 21:

                if(driveTrain.driveStraight(Directions.FORWARD, 7)) {
                    foundation.lUp();
                    step++;
                }
                break;
            //Strafes away from foundation
            case 22:
                    if(driveTrain.strafeDrive(Directions.RIGHT, 4)) {
                        claw.release();
                        claw.halfUp();
                        step++;
                    }

                break;
            //Turns to line
            case 23:
                if(driveTrain.rotate(180)) {
                    step++;
                }
                break;
            //Drives to line
            case 24:
                if (driveTrain.driveToLine(color, Directions.FORWARD)) {
                    step++;
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
