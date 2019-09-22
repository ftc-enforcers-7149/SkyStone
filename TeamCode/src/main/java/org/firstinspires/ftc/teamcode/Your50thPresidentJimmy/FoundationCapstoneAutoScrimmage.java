package org.firstinspires.ftc.teamcode.Your50thPresidentJimmy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SkyStonev1_1;

        @Autonomous(name="auto 2 CAPSTONE")
        public class FoundationCapstoneAutoScrimmage extends SkyStonev1_1 {
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
                    case 1: Rotation (270);
                        break;
                        //FULL SPEED

                    case 2: driveStraight("backward", 5);
                        break;
                    case 3: Rotation (180);
                        break;
                    case 4:driveStraight("forward", 34);
                        break;
                    case 5:Rotation(90);
                        break;
                    case 6:driveStraight("backward", 23);
                        break;
                }
                step++;

            }

            public void stop() {
                super.stop();
            }
        }
