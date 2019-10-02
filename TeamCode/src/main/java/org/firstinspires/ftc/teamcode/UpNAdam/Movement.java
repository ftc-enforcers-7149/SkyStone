package org.firstinspires.ftc.teamcode.UpNAdam;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Movement {
    private DcMotor bLeft,bRight,fLeft,fRight;

    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1;
    private static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    public Movement(){

    }
    public Movement(DcMotor motor1,DcMotor motor2,DcMotor motor3,DcMotor motor4){
        bLeft=motor1;
        bRight=motor2;
        fLeft=motor3;
        fRight=motor4;
    }

    /**
     * drives inputted distance
     * @param direction direction of driving. "backward" to go backward
     * @param distance distance driving in inches
     */
    public void driveStraight(String direction, double distance) {
        resetEncoderWithoutEncoder();
        int mDirection = 1;
        if (direction.equals("backward")) {
            mDirection = -1;
        }

        double power=0.6*mDirection;
        double cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;

        while(cPosition < distance){
            cPosition=fRight.getCurrentPosition()/COUNTS_PER_INCH*mDirection;
            if(distance-Math.abs(cPosition)<20){
                power=0.2*mDirection;
            }
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }

        fLeft.setPower(0);
        fRight.setPower(0);
        bLeft.setPower(0);
        bRight.setPower(0);
    }

    /**
     * Resets drive encoders without running using encoders
     */
    public void resetEncoderWithoutEncoder(){
        bRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
