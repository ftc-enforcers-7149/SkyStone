package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name = "Red Max Stone V1")
public class RedMaxStoneV1 extends ParentInit {

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
                if (driveTrain.driveColorDist(color, Directions.FORWARD, 20, 0)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Waits and puts foundation grabber up
            case 7:
                if(driveTrain.delay(500)) {
                    foundation.lUp();
                    step++;
                }
                break;
            //Drives back to second block
            case 8:
                if(position == Positions.CENTER) {
                    offset = 6.6;
                }
                if (driveTrain.driveColorDist(color, Directions.BACKWARD,30 + offset, 0)) {
                    step++;
                }
                break;
            //Strafes to block
            case 9:
                if (driveTrain.strafeDrive(Directions.LEFT,6, 0.6, 0)) {
                    step++;
                }
                break;
            case 10:
                if (driveTrain.driveStraight(Directions.FORWARD,2,0,0.2)) {
                    foundation.lHalf();
                    step++;
                }
                break;
            //Strafes away from block
            case 11:
                if (driveTrain.strafeDrive(Directions.RIGHT,12,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            //Drives forward to foundation
            case 12:
                if (driveTrain.driveColorDist(color, Directions.FORWARD,20 + offset, 0)) {
                    foundation.lDown();
                    step++;
                }
                break;
            case 13:
                foundation.lUp();
                step++;

        }
    }

    public void stop() {
        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }
}
