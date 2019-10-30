package org.firstinspires.ftc.teamcode.CollisionAvoidance;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@TeleOp(name = "Detect Movement")
public class TestMovementDetection extends ParentInit {

    //Special objects
    MovementDetectionClass detection;

    //Variables for motor powers and update time
    double fL, fR, bL, bR;
    double lim;
    double prevTime;

    public void init() {
        //Initialize motors, servos, and imu
        super.init();

        //Initialize obstacle detection and drive system objects
        detection = new MovementDetectionClass(hardwareMap, "distanceC", "distanceR", "distanceL", fLeft, fRight, bLeft, bRight);

        //Initialize needed variables
        lim=0.5;
    }

    public void start() {
        prevTime = System.currentTimeMillis();
    }

    public void loop() {
        if (System.currentTimeMillis() - prevTime > 100) {
            detection.update();

            prevTime += 100;
        }
        boolean isFrontMoving = detection.isFrontMoving();
        boolean isFront = detection.isFrontClose();
        boolean isLeft = detection.isLeftClose();
        boolean isRight = detection.isRightClose();
        String raw = detection.rawData();

        telemetry.addData("Moving in front? ", isFrontMoving);
        telemetry.addData("Moving in front? ", isFront);
        telemetry.addData("Is left close? ", isLeft);
        telemetry.addData("Is right close? ", isRight);
        telemetry.addLine(raw);
        Log.i("Raw Data: ", raw);

        fL = 0;         //Begin motor power logic by setting velocities to 0
        fR = 0;
        bL = 0;
        bR = 0;
        if (isFront) {
            if (isRight) {
                fL -= 0.3;
                fR += 0.3;
                bL += 0.3;
                bR -= 0.3;
            }
            else {
                fL += 0.3;
                fR -= 0.3;
                bL -= 0.3;
                bR += 0.3;
            }

            fL -= 0.2;
            fR -= 0.2;
            bL -= 0.2;
            bR -= 0.2;
        }
        else if (isFrontMoving) {
            fL -= 0.1;
            fR -= 0.1;
            bL -= 0.1;
            bR -= 0.1;
        }
        else {
            if (isLeft) {
                fL += 0.2;
                fR -= 0.2;
                bL -= 0.2;
                bR += 0.2;
            } else if (isRight) {
                fL -= 0.2;
                fR += 0.2;
                bL += 0.2;
                bR -= 0.2;
            }
        }
        fL += 0.3;
        fR += 0.3;
        bL += 0.3;
        bR += 0.3;

        //Getting the max value can assure that no motor will be set to a value above a certain point.
        double max = Math.max(Math.max(Math.abs(fL), Math.abs(fR)), Math.max(Math.abs(bL), Math.abs(bR)));

        //In this case, no motor can go above lim power by scaling them all down if such a thing might occur.
        if (max > lim) {
            fL /= max * (1 / lim);
            fR /= max * (1 / lim);
            bL /= max * (1 / lim);
            bR /= max * (1 / lim);
        }

        detection.fLeft.setPower(fL);
        detection.fRight.setPower(fR);
        detection.bLeft.setPower(bL);
        detection.bRight.setPower(bR);
    }
}