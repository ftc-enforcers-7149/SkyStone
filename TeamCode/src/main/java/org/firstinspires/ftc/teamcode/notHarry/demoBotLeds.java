package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSystems.Headless;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;

import com.qualcomm.robotcore.hardware.DistanceSensor;
//@TeleOp(name = "demoBotLeds")
public class demoBotLeds extends OpMode {

      DistanceSensor autoGrab;

      Headless driveSystem;
      Gyroscope gyro;

      DcMotor fLeft, fRight, bLeft, bRight;


    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
        public void init(){
            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(pattern);

            fLeft = hardwareMap.dcMotor.get("fLeft");
            fRight = hardwareMap.dcMotor.get("fRight");
            bLeft = hardwareMap.dcMotor.get("bLeft");
            bRight = hardwareMap.dcMotor.get("bRight");

            autoGrab=hardwareMap.get(DistanceSensor.class,"autoGrab");

            fLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            fRight.setDirection(DcMotorSimple.Direction.REVERSE);
            bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            bRight.setDirection(DcMotorSimple.Direction.REVERSE);

            //Drive System initialization
            //gyro = new Gyroscope(telemetry, hardwareMap);
            driveSystem = new Headless(gyro, fLeft, fRight, bLeft, bRight, false);//false);




        }
        public void loop(){
            //blinkinLedDriver.setPattern(pattern);
            telemetry.addData("range", String.format("%.01f in", autoGrab.getDistance(DistanceUnit.INCH)));
            telemetry.update();

            if(autoGrab.getDistance(DistanceUnit.INCH) < 5){
                pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
            }else if (autoGrab.getDistance(DistanceUnit.INCH) > 5){
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            }else{
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
            blinkinLedDriver.setPattern(pattern);

            //driveSystem.drive(gamepad1);

        }
        public void stop(){
            driveSystem.stop();
        }

    }


