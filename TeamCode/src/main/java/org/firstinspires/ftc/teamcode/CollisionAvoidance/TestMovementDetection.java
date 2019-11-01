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
        if (isFront) {     //If obstacle is detected close to front, move faster
            //Avoid obstacle by moving side to side
            if (isRight) {      //Move left if obstacle on right
                fL -= 0.3;
                fR += 0.3;
                bL += 0.3;
                bR -= 0.3;
            }
            else {              //Move right for any other case
                fL += 0.3;
                fR -= 0.3;
                bL -= 0.3;
                bR += 0.3;
            }

            fL -= 0.25;          //Slow down forward motion
            fR -= 0.25;
            bL -= 0.25;
            bR -= 0.25;
        }
        else if (isFrontMoving) {       //Slow down a bit for an object moving towards the robot
            fL -= 0.1;
            fR -= 0.1;
            bL -= 0.1;
            bR -= 0.1;
        }
        else {              //If there is no object / obstacle in front, handle for left and right
            if (isLeft) {         //If obstacle on left, move right
                fL += 0.2;
                fR -= 0.2;
                bL -= 0.2;
                bR += 0.2;
            } else if (isRight) {   //If obstacle on right, move left
                fL -= 0.2;
                fR += 0.2;
                bL += 0.2;
                bR -= 0.2;
            }
        }
        fL += 0.3;        //Move forward overall. Affected by above statements, so will only drive at this speed if no obstacles / objects detected
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