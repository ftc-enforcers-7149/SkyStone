package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@Autonomous(name = "Strafe Avoiding")
public class StrafeAvoidance extends ParentInit {

    int step = 0;

    public void init() {
        super.init();
        claw.up();
    }

    public void loop() {
        telemetry.addData("Step: ", step);
        telemetry.addData("DistanceL: ", distanceL.getDistance(DistanceUnit.CM));
        telemetry.addData("DistanceR: ", distanceR.getDistance(DistanceUnit.CM));

        switch (step) {
            case 0:
                if (driveTrain.driveAvoid(distanceL, distanceR, 60, 0, 0.6)) {
                    step++;
                }
                break;
            case 1:
                if (driveTrain.rotate(180)) {
                    step++;
                }
                break;
            case 2:
                //Check left sensor -> strafe right
                if (driveTrain.driveAvoid(distanceL, distanceR, 60, 180, 0.6)) {
                    step++;
                }
                break;
            case 3:
                if (driveTrain.rotate(0)) {
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
