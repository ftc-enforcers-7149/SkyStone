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
package org.firstinspires.ftc.teamcode.AutonomousPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Foundation;

@Autonomous(name = "Red Foundation Park")
//@Disabled                            // Comment this out to add to the opmode list
public class RedFoundationPark extends OpMode {

    public Servo lArm, rArm, lGrab, rGrab, lFound, rFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;

    DriveTrain driveTrain;
    Foundation foundation;

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
        driveTrain=new DriveTrain(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
        foundation=new Foundation(lFound,rFound);
    }

    // Loop and update the dashboard//
    public void loop() {
        switch(step){
            case 0:
                lArm.setPosition(0.1);
                rArm.setPosition(0.05);
                break;
            case 1:
                driveTrain.driveStraight("forward",47);//50
                break;
            case 2:
                driveTrain.strafeSeconds(750,"right");
                break;
            case 3:
                foundation.down();
                break;
            case 4:
                driveTrain.delay(1000);
                break;
            case 5:
                driveTrain.driveStraight("backward",3);
            case 6:
                driveTrain.strafeSeconds(1000,"left");
            case 7:
                driveTrain.rotation(295,0.45);
                //driveTrain.driveStraight("backward", 35, 0.7,0.7);
                break;
            case 8:
                driveTrain.strafeSeconds(4000,"right");
            case 9:
                foundation.up();
                break;
            case 10:
                //driveTrain.strafeSeconds(2500,"left");
                break;
            case 11:
                //driveTrain.rotation(270);

        }
        step++;
    }

}
