/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.AutonomousPrograms.V1;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV1;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

//@Autonomous(name = "Red FoundationV1 Park")
@Disabled                       // Comment this out to add to the opmode list
public class RedFoundationParkOldV1 extends OpMode {

    public Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;
    Gyroscope gyro;

    DriveTrainV1 driveTrainV1;
    FoundationV1 foundationV1;

    int step=0;

    public void init() {

        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        lFound = hardwareMap.servo.get("lFound");
        rFound = hardwareMap.servo.get("rFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.REVERSE);
        rArm.setDirection(Servo.Direction.FORWARD);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        lFound.setDirection(Servo.Direction.REVERSE);
        rFound.setDirection(Servo.Direction.FORWARD);

        //Servos up
        rFound.setPosition(0);
        lFound.setPosition(0);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void start(){
        driveTrainV1 =new DriveTrainV1(telemetry,fLeft,fRight,bLeft,bRight,gyro);
        foundationV1 =new FoundationV1(lFound,rFound);
    }

    // Loop and update the dashboard//
    public void loop() {
        switch(step){
            case 0://driveTrainV1.delay(3000);
                break;
            case 1://driveTrainV1.delay(3000);
                break;
            case 2://driveTrainV1.delay(4000);
                break;
            case 3://driveTrainV1.delay(3000);
                break;
            case 4:
                lArm.setPosition(0.1);
                rArm.setPosition(0.05);
                break;
            case 5:
                driveTrainV1.driveStraight("forward",47);//50
                break;
            case 6:
                driveTrainV1.strafeSeconds(500,"right");
                break;
            case 7:
                foundationV1.down();
                break;
            case 8:
                driveTrainV1.delay(1000);
                break;
            case 9:
                //driveTrainV1.driveStraight("backward",3);
            case 10:
                driveTrainV1.strafeSeconds(250,"left");
            case 11:
                driveTrainV1.simpleTurn(-45,0.45);//driveTrainV1.simpleRotateRed(295,0.35);//0.45
                //driveTrainV1.driveStraight("backward", 35, 0.7,0.7);
                break;
            case 12:
                driveTrainV1.strafeSeconds(3000,"right");
            case 13:
                foundationV1.up();
                break;
            case 14:
                driveTrainV1.strafeSeconds(250,"left");
                break;
            case 15:
                driveTrainV1.driveStraight("backward", 28);
                break;
            case 16:
                driveTrainV1.rotation(270);
                break;
            case 17:
                driveTrainV1.driveStraight("backward", 12);
                break;
            case 18:
                //driveTrainV1.rotation(180);
                break;

        }
        step++;
    }

}
