package org.firstinspires.ftc.teamcode.notHarry;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode._Reference.LEDTest;

public class ledLiftTest {


    DcMotor lift;
    DistanceSensor distanceLift;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    double dL ;

    boolean pressP, pressM, levelPlus, levelMinus, liftPress;

    int level;

    public ledLiftTest(DcMotor lift, DistanceSensor distanceLift) {
        this.lift = lift;
        this.distanceLift = distanceLift;
    }


    /**
     * Uses logic from AutoLiftTest
     * Contains button press logic
     * Uses switch statement to know how to go to each level
     * @param gamepad1
     */
    public void liftSet(Gamepad gamepad1) {
        blinkinLedDriver.setPattern(pattern);
        levelPlus = gamepad1.dpad_up;
        levelMinus = gamepad1.dpad_down;

        if(!pressP){
            if (levelPlus) {
                if (level < 8){
                    level++;
                    liftPress=true;
                }
                pressP = true;
            }
        }
        else{
            if (!levelPlus){
                pressP = false;
            }
        }

        if(!pressM){
            if (levelMinus) {
                if(level > 0){
                    level--;
                    liftPress=true;
                }
                pressM = true;
            }
        }
        else{
            if (!levelMinus){
                pressM = false;
            }
        }

        if(liftPress) {
            switch (level) {
                case 0:
                    if(distanceLift.getDistance(DistanceUnit.CM) > 1) { //3
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0);
                        liftPress = false;
                    }
                    break;
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 10) {  //10.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 11) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 20) { //20.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 21) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 30) { //30.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 31) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 40) { //40.5
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 41) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 55) {   //54
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 56) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 6:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 63) {   //62
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 64) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 7:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 73) {   //72
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 74) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 8:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 82) {   //81
                        lift.setPower(0.8);
                    }
                    else if(distanceLift.getDistance(DistanceUnit.CM) > 83) {
                        lift.setPower(-0.4);
                    }
                    else{
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
            }
        }
    }

    public int getLevel() {
        return level;
    }
}
