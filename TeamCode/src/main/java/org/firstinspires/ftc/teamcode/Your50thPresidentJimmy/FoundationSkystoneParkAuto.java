package org.firstinspires.ftc.teamcode.Your50thPresidentJimmy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SkyStonev1_1;

        @Autonomous(name="auto 3 Skystone")
        public class FoundationSkystoneParkAuto extends SkyStonev1_1 {
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
                    //Change speed initially to make sure capstone does not dislocate
                    case 0:driveStraight("backward", 50);
                        break;
                    case 1:driveStraight("forward", 24.5);
                        break;
                    case 2: Rotation (270);
                        break;
                    case 3:driveStraight("forward", 56);
                        break;
                    case 4:driveStraight("backward", 33);
                        break;
                }
                step++;

            }
            public void stop() {
                super.stop();
            }
        }
