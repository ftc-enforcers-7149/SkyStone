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
package org.firstinspires.ftc.teamcode.AutonomousPrograms.V2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.FoundationV2;

@Autonomous(name = "Blue Foundation ParkV2")
//@Disabled                            // Comment this out to add to the opmode list
public class BlueFoundationParkV2 extends OpMode {

    public Servo lArm, rArm, lGrab, rGrab;
    Servo fLFound, fRFound, bLFound, bRFound;
    public DcMotor fRight,fLeft,bRight,bLeft,lift;
    ColorSensor color;

    DriveTrainV1 driveTrainV1;
    FoundationV2 foundation;
    Claw claw;

    int step=0;

    public void init() {

        //Servos
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lGrab = hardwareMap.servo.get("lGrab");
        rGrab = hardwareMap.servo.get("rGrab");
        fLFound = hardwareMap.servo.get("fLFound");
        fRFound = hardwareMap.servo.get("fRFound");
        bLFound = hardwareMap.servo.get("bLFound");
        bRFound = hardwareMap.servo.get("bRFound");
        //Drive motors
        fLeft = hardwareMap.dcMotor.get("fLeft");
        fRight = hardwareMap.dcMotor.get("fRight");
        bLeft = hardwareMap.dcMotor.get("bLeft");
        bRight = hardwareMap.dcMotor.get("bRight");
        lift = hardwareMap.dcMotor.get("lift");

        //Color sensor
        color = hardwareMap.colorSensor.get("color");

        //direction of motors
        fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        //direction of servos
        lArm.setDirection(Servo.Direction.FORWARD);
        rArm.setDirection(Servo.Direction.REVERSE);
        lGrab.setDirection(Servo.Direction.REVERSE);
        rGrab.setDirection(Servo.Direction.FORWARD);
        fLFound.setDirection(Servo.Direction.REVERSE);
        fRFound.setDirection(Servo.Direction.FORWARD);
        bLFound.setDirection(Servo.Direction.FORWARD);
        bRFound.setDirection(Servo.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lArm.setPosition(0.95);
        rArm.setPosition(0.81);

        fLFound.setPosition(1);
        bLFound.setPosition(1);

        fRFound.setPosition(1);
        bRFound.setPosition(1);

    }
    public void start(){
        //driveTrainV1 =new DriveTrainV1(hardwareMap,telemetry,fLeft,fRight,bLeft,bRight);
        foundation =new FoundationV2(fLFound,fRFound,bLFound,bRFound);
        claw=new Claw(lArm,rArm,lGrab,rGrab);
    }

    // Loop and update the dashboard
    public void loop() {
        switch(step){
            case 1:
                //foundation.lDown();
                break;
            case 2:
                driveTrainV1.driveStraight("backward",47);//50
                break;
            case 3:
                driveTrainV1.strafeSeconds(750,"right");
                break;
            case 4:
                foundation.rDown();
            break;
            case 5:
                driveTrainV1.delay(1000);
                break;
            case 6:

            case 7:
                driveTrainV1.strafeSeconds(250,"left");
            case 8:
                driveTrainV1.simpleTurn(45,0.45);//0.45
                //was already commented out: driveTrainV1.driveStraight("backward", 35, 0.7,0.7);
                break;
            case 9:
                driveTrainV1.strafeSeconds(3000,"right");
            case 10:
                foundation.rUp();
                break;
            case 11:
                driveTrainV1.strafeSeconds(250,"left");
                break;
            case 12:
                driveTrainV1.driveStraight("forward", 28);
                break;
            case 13:
                driveTrainV1.rotation(90);
                break;
            case 14:
                driveTrainV1.driveToLine(color, "blue", "forward");
                break;
        }
        step++;
        //Kpop = jMetal
    }

    public void stop() {
    }
}
