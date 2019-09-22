package org.firstinspires.ftc.teamcode.Your50thPresidentJimmy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SkyStonev1_1;

@Autonomous(name="auto 1")
public class FoundationAutoRed extends SkyStonev1_1 {
    DcMotor fLeft, fRight, bLeft, bRight;
    DistanceSensor distanceL, distanceR, distanceC;
    BNO055IMU imu;
    Orientation angles;

    int step=0;
    public void init() {
        super.init();
    }

    public void loop() {
        switch(step){
            case 0:driveStraight("forward", 50);
                break;
            case 1:driveStraight("backward", 34);
                break;
            case 2:Rotation(90);
                break;
            case 3:driveStraight("forward", 20);
                break;
        }
        step++;

    }

    public void stop() {
      super.stop();
    }
}
