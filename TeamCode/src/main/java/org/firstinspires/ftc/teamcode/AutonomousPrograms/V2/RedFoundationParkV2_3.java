package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name = "Red Foundation Park V2_3")
public class RedFoundationParkV2_3 extends ParentInit {

    OdometryPosition.Direction direction;
    Positions position=Positions.RIGHT;

    int step = 0;

    public void init() {

        super.init();

        direction = OdometryPosition.Direction.FORWARD;

    }

    public void init_loop() { super.init_loop(); }

    public void start() {
        claw.up();
    }

    public void loop() {
        telemetry.addData("delta: ",gyroscope.getDelta(90,gyroscope.getRawYaw()));
        telemetry.addData("heading: ",gyroscope.getYaw());
        telemetry.addData("step: ",step);

        switch (step) {
            case 0:
                if (driveTrain.driveStraight(Directions.FORWARD, 5)) {
                    step++;
                }
                break;
            case 1:
                if (driveTrain.strafeDrive(Directions.LEFT,35,0.4)) {
                    foundation.lDown();
                    step++;
                }
                break;
            case 2:
                if(driveTrain.delay(1250)){
                    step++;
                }
                break;
            case 3:
                if (driveTrain.foundationTurn(90)){
                    step++;
                }
                break;
            case 4:
                if(driveTrain.strafeSeconds(Directions.LEFT,1500,0.7)){
                    step++;
                }
                break;
            case 5:
                if(driveTrain.driveStraight(Directions.FORWARD, 2)) {
                    foundation.lUp();
                    step++;
                }
                break;
            case 6:
                if(driveTrain.strafeDrive(Directions.RIGHT, 4)) {
                    claw.halfUp();
                    step++;
                }
                break;
            case 7:
                if(driveTrain.rotate(270, .3)) {
                    step++;
                }
                break;
            case 8:
                if (driveTrain.driveStraight(Directions.BACKWARD, 20)) {
                    step++;
                }
                break;
            case 9:
                if (driveTrain.strafeDrive(Directions.LEFT, 30)) {
                    step++;
                }
                else if (color.red()>80) {
                    step++;
                }
                break;
            case 10:

                fLeft.setPower(0);
                fRight.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
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
