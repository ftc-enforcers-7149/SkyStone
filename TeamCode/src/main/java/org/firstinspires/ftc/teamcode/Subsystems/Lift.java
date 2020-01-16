package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Lift {

    DcMotor lift;
    DistanceSensor distanceLift;

    boolean pressP, pressM, levelPlus, levelMinus, liftPress;

    int level;

    public Lift(DcMotor lift, DistanceSensor distanceLift) {
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
        levelPlus = gamepad1.dpad_up;
        levelMinus = gamepad1.dpad_down;

        if (!pressP) {
            if (levelPlus) {
                if (level < 9) {
                    level++;
                    liftPress = true;
                }
                pressP = true;
            }

        } else {
            if (!levelPlus) {
                pressP = false;
            }
        }

        if (!pressM) {
            if (levelMinus) {
                if (level > 0) {
                    level--;
                    liftPress = true;
                }
                pressM = true;

            }

        } else {
            if (!levelMinus) {
                pressM = false;
            }
        }

        if (liftPress) {
            switch (level) {
                case 0:
                    if (distanceLift.getDistance(DistanceUnit.CM) > 1.5) { //3
                        lift.setPower(-0.4);
                    } else {
                        lift.setPower(0);
                        liftPress = false;
                    }
                    break;
                case 1:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 4) {  //10.5
                        lift.setPower(0.45);//0.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 7) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 2:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 13) { //20.5
                        lift.setPower(0.45);//.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 16) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 3:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 24) { //30.5
                        lift.setPower(0.45);//.67
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 26) {
                        lift.setPower(-0.2);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 4:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 35) { //40.5
                        lift.setPower(0.45);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 38) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 5:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 45.5) {   //54
                        lift.setPower(0.47);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 47.5) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 6:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 56) {   //62
                        lift.setPower(0.49);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 58) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 7:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 66) {   //62
                        lift.setPower(0.49);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 68) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 8:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 77) {   //72
                        lift.setPower(0.51);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 79) {
                        lift.setPower(0);
                    } else {
                        lift.setPower(0.1);
                        liftPress = false;
                    }
                    break;
                case 9:
                    if (distanceLift.getDistance(DistanceUnit.CM) < 87) {   //81
                        lift.setPower(0.51);
                    } else if (distanceLift.getDistance(DistanceUnit.CM) > 90) {
                        lift.setPower(0);
                    } else {
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
