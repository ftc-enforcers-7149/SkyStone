package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Misc.Position;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Directions;
import org.firstinspires.ftc.teamcode.Subsystems.Enums.Positions;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;

public class DriveTrainV4 {
    //IMU variables
    private Gyroscope gyro;

    private DcMotor fLeft, fRight, bLeft, bRight;

    private OdometryPosition oP;

    private Telemetry telemetry;

    //used for encoders
    private static final double     EXTERNAL_GEARING        = 1.5;    //From sprockets
    private static final double     COUNTS_PER_MOTOR_REV    = 753.2 ;  //28  // eg: AndyMark NeverRest40 Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = ((COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415))/EXTERNAL_GEARING;

    private double endTime, last_time=0;
    private double last_dist=0;

    private double x, y;

    private boolean last_ang=true;
    private double initAngle;
    private boolean driveColor=true;

    /**
     * Main constructor
     * @param telemetry telemetry
     * @param fLeft fLeft
     * @param fRight fRight
     * @param bLeft bLeft
     * @param bRight bRight
     * @param gyro gyro
     */
    public DriveTrainV4(HardwareMap hardwareMap, Telemetry telemetry, DcMotor fLeft, DcMotor fRight, DcMotor bLeft, DcMotor bRight, Gyroscope gyro){
        this.fLeft = fLeft;
        this.fRight = fRight;
        this.bLeft = bLeft;
        this.bRight = bRight;
        this.gyro=gyro;
        this.telemetry=telemetry;

        oP = new OdometryPosition(hardwareMap, "encX", "encY", 0, 0, gyro, fLeft, fRight, bLeft, bRight);
        oP.reverseY();
    }

    public void updateOdom(OdometryPosition.Direction dir) {
        oP.updatePosition(dir);
    }

    /**
     * Drives to any point
     * @param x X coord
     * @param y Y coord
     * @param power Power limit for motors
     * @return
     */
    public boolean driveToPoint(double x, double y, double power) {
        if (oP.driveToPoint(x, y, power, telemetry)) {
            return true;
        }

        return false;
    }

    /**
     * Drives to any point
     * Auto corrects angle for strafing
     * @param x X coord
     * @param y Y coord
     * @param power Power limit for motors
     * @return
     */
    public boolean driveToPoint(double x, double y, double power, double ang) {
        if (oP.driveToPoint(x, y, power, ang, telemetry)) {
            return true;
        }

        return false;
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

        oP.resetEncoderWithoutEncoder();
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean driveStraight(Directions direction, double dist) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.BACKWARD) {
            mDirection = -1;
        }
        double power=0.6*mDirection;

        //converts current position into inches
        double cPosition=oP.getPositionY()*mDirection;

        if(cPosition < last_dist){
            if(last_dist-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }
            fLeft.setPower(power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(power);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean driveStraight(Directions direction, double dist, double ang) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.BACKWARD) {
            mDirection = -1;
        }
        double power=0.6*mDirection;

        //converts current position into inches
        double cPosition=oP.getPositionY()*mDirection;

        double delta = gyro.getDelta(ang, gyro.getRawYaw());
        double turnL = 0;
        double turnR = 0;

        if(cPosition < last_dist){
            if(delta < 1){
                //power -= 0.1;
                turnR = 0.075;
            }
            else if(delta > -1){
                //power -= 0.1;
                turnL = 0.075;
            }

            fLeft.setPower(power - turnL);
            fRight.setPower(power - turnR);
            bLeft.setPower(power - turnL);
            bRight.setPower(power - turnR);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean driveStraight(Directions direction, double dist, double ang, double power) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.BACKWARD) {
            mDirection = -1;
        }
        power*=mDirection;

        //converts current position into inches
        double cPosition=oP.getPositionY()*mDirection;

        double delta = gyro.getDelta(ang, gyro.getRawYaw());
        double turnL = 0;
        double turnR = 0;

        if(cPosition < last_dist){
            if(delta < 1){
                //power -= 0.1;
                turnR = 0.075;
            }
            else if(delta > -1){
                //power -= 0.1;
                turnL = 0.075;
            }

            fLeft.setPower(power - turnL);
            fRight.setPower(power - turnR);
            bLeft.setPower(power - turnL);
            bRight.setPower(power - turnR);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean foundationDrive(Directions direction, Positions position, double dist, double power) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.BACKWARD) {
            mDirection = -1;
        }

        //converts current position into inches
        double cPosition=oP.getPositionY()*mDirection;


        if(cPosition < last_dist){
          if(position==Positions.RIGHT){
              fLeft.setPower(0.4 * mDirection);
              fRight.setPower(power * mDirection);
              bLeft.setPower(0.4 * mDirection);
              bRight.setPower(power * mDirection);
          }
          else{
              fRight.setPower(0.4 * mDirection);
              fLeft.setPower(power * mDirection);
              bRight.setPower(0.4 * mDirection);
              bLeft.setPower(power * mDirection);
          }
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    public boolean driveAvoid(DistanceSensor distanceL, DistanceSensor distanceR, double dist, double ang, double power) {
        double strafePower = power / 2;
        //Check left sensor -> strafe right
        if (distanceL.getDistance(DistanceUnit.CM) < (dist * 200) / 3) { //Example distance: 0.6 speed -> 40 distance
            fLeft.setPower(strafePower);
            fRight.setPower(-strafePower);
            bLeft.setPower(-strafePower);
            bRight.setPower(strafePower);
        }
        //Check right sensor -> strafe left
        else if (distanceR.getDistance(DistanceUnit.CM) < (dist * 200) / 3) {
            fLeft.setPower(-strafePower);
            fRight.setPower(strafePower);
            bLeft.setPower(strafePower);
            bRight.setPower(-strafePower);
        }
        //If nothing detected -> continue driving
        else if (driveStraight(Directions.FORWARD, dist, ang, power)) {
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean strafeDrive(Directions direction, double dist) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.RIGHT) {
            mDirection = -1;
        }
        double power=0.6*mDirection;

        //converts current position into inches
        double cPosition=oP.getPositionX()*mDirection;

        if(cPosition < last_dist){
            /*if(last_dist-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }*/
            fLeft.setPower(-power);
            fRight.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(-power);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches)
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean strafeDrive(Directions direction, double dist, double power) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.RIGHT) {
            mDirection = -1;
        }


        //converts current position into inches
        double cPosition=oP.getPositionX()*mDirection;

        if(cPosition < last_dist){
            /*if(last_dist-Math.abs(cPosition)<20){
                power=0.4*mDirection;
            }*/
            fLeft.setPower(-power*mDirection);
            fRight.setPower(power*mDirection);
            bLeft.setPower(power*mDirection);
            bRight.setPower(-power*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * drives inputted distance(inches) with angle correct
     * @param direction direction of driving.
     * @param dist distance to drive.
     */
    public boolean strafeDrive(Directions direction, double dist, double power, double ang) {
        if (last_dist != dist) {
            last_dist = dist;
            resetEncoderWithoutEncoder();
        }
        //sets direction of motors
        int mDirection = 1;
        if (direction == Directions.RIGHT) {
            mDirection = -1;
        }


        //converts current position into inches
        double cPosition=oP.getPositionX()*mDirection;

        double delta = gyro.getDelta(ang, gyro.getRawYaw());
        double turnL = 0;
        double turnR = 0;

        if(cPosition < last_dist){
            if(delta < 1){
                //power -= 0.1;
                turnR = 0.075;
            }
            else if(delta > -1){
                //power -= 0.1;
                turnL = 0.075;
            }
            fLeft.setPower(-power*mDirection);
            fRight.setPower(power*mDirection);
            bLeft.setPower(power*mDirection);
            bRight.setPower(-power*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_dist = 0;
            return true;
        }

        return false;
    }

    /**
     * Strafes inputted distance(inches)
     * @param direction direction of strafing.
     * @param dist distance to strafe.
     * @param power Limit for power to drive
     */
    public boolean driveStrafe(Directions direction, double dist, double power) {
        if (last_dist != dist) {
            last_dist = dist;

            double offsetAngle;

            if (direction == Directions.RIGHT) {
                offsetAngle = 90;
            }
            else {
                offsetAngle = -90;
            }

            double angle = Math.toRadians(oP.cvtDegrees(oP.getHeading() + offsetAngle));

            if (direction == Directions.RIGHT) {
                x = dist * Math.cos(angle);
                y = dist * Math.sin(angle);
            } else {
                x = -dist * Math.cos(angle);
                y = -dist * Math.sin(angle);
            }
        }

        /*if (oP.driveToPoint(x, y, power, telemetry)) {
            last_dist = 0;

            return true;
        }*/

        return false;
    }

    /**
     * strafes for a given time
     * @param direction "left" for left "right" for right
     */
    public boolean strafeSecondsBlue(Directions direction, double time,double power){
        if (last_time != time) {
            last_time = time;
            endTime = System.currentTimeMillis() + time;
        }
        //sets direction strafing
        int mDirection=-1;
        if(direction == Directions.RIGHT){
            mDirection=1;
        }

        if (System.currentTimeMillis()<endTime){
            fLeft.setPower((power/2)*mDirection);
            fRight.setPower(-power*mDirection);
            bLeft.setPower(-power*mDirection);
            bRight.setPower(power*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * strafes for a given time
     * @param direction "left" for left "right" for right
     */
    public boolean strafeSecondsRed(Directions direction, double time,double power){
        if (last_time != time) {
            last_time = time;
            endTime = System.currentTimeMillis() + time;
        }
        //sets direction strafing
        int mDirection=-1;
        if(direction == Directions.RIGHT){
            mDirection=1;
        }

        if (System.currentTimeMillis()<endTime){
            fLeft.setPower((power)*mDirection);
            fRight.setPower(-power*mDirection);
            bLeft.setPower(-power*mDirection);
            bRight.setPower((power/2)*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * strafes for a given time
     * @param direction "left" for left "right" for right
     */
    public boolean strafeSeconds(Directions direction, double time,double power){
        if (last_time != time) {
            last_time = time;
            endTime = System.currentTimeMillis() + time;
        }
        //sets direction strafing
        int mDirection=-1;
        if(direction == Directions.RIGHT){
            mDirection=1;
        }

        if (System.currentTimeMillis()<endTime){
            fLeft.setPower((power)*mDirection);
            fRight.setPower(-power*mDirection);
            bLeft.setPower(-power*mDirection);
            bRight.setPower(power*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            return true;
        }

        return false;
    }
    /**
     * Gets the shortest distance to destAngle
     * and drives motors at a proportional speed
     * to slow down as it gets closer
     * Min Speed = 0.1
     * Max Speed = 0.8
     * @param destAngle Destination angle
     * @return
     */
    public boolean rotate(double destAngle) {
        if (last_ang) {
            initAngle=360-gyro.getYaw();
            last_ang=false;
        }
        double speed=0, min=0.175;

        //Get current heading
        double heading = 360-gyro.getYaw();
        double delta = gyro.getDelta(destAngle, heading);
        boolean left=false;
        boolean done=false;


        if(gyro.getDelta(destAngle, initAngle) > 0){
            left=false;
        }
        else{
            left=true;
        }


        if(left){
            if(delta > 0){
                done=true;
            }
        }
        else{
            if(delta < 0){
                done=true;
            }
        }

        //If heading is not at destination
        if (!done) {
            //Get shortest distance to angle


            //Calculate speed (Linear calculation)
            //Farthest (180 away) : 0.8
            //Closest (0 away) : 0.1
            if (delta > 0) {
                speed = (delta / 257.144) + 0.1;
            }
            else {
                speed = (-delta / 257.144) + 0.1;
                speed = -speed;
            }

            if (speed > 0 && speed < min) {
                speed = min;
            }
            else if (speed < 0 && speed > -min) {
                speed = -min;
            }

            telemetry.addData("Angle: ", delta);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Delta: ",delta);

            //Drive the motors so the robot turns
            fLeft.setPower(speed);
            fRight.setPower(-speed);
            bLeft.setPower(speed);
            bRight.setPower(-speed);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_ang = true;

            return true;
        }

        return false;
    }

    /**
     * Gets the shortest distance to destAngle
     * and drives motors at a proportional speed
     * to slow down as it gets closer
     * Min Speed = 0.1
     * Max Speed = 0.8
     * @param destAngle Destination angle
     * @return
     */
    public boolean rotate(double destAngle,double speed) {
        if (last_ang) {
            initAngle=360-gyro.getYaw();
            last_ang=false;
        }

        //Get current heading
        double heading = 360-gyro.getYaw();
        double delta = gyro.getDelta(destAngle, heading);
        boolean left;
        boolean done=false;
        double mDirection;


        if(gyro.getDelta(destAngle, initAngle) > 0){
            left=false;
            mDirection=1;
        }
        else{
            left=true;
            mDirection=-1;
        }


        if(left){
            if(delta > 0){
                done=true;
            }
        }
        else{
            if(delta < 0){
                done=true;
            }
        }

        //If heading is not at destination
        if (!done) {

            telemetry.addData("Angle: ", delta);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Delta: ",delta);

            //Drive the motors so the robot turns
            fLeft.setPower(speed*mDirection);
            fRight.setPower(-speed*mDirection);
            bLeft.setPower(speed*mDirection);
            bRight.setPower(-speed*mDirection);
        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_ang = true;

            return true;
        }

        return false;
    }

    /**
     * Gets the shortest distance to destAngle
     * and drives motors at a proportional speed
     * to slow down as it gets closer
     * Min Speed = 0.1
     * Max Speed = 0.8
     * @param destAngle Destination angle
     * @return
     */
    public boolean foundationTurn(double destAngle) {
        if (last_ang) {
            initAngle=360-gyro.getYaw();
            last_ang=false;
        }

        //Get current heading
        double heading = 360-gyro.getYaw();
        double delta = gyro.getDelta(destAngle, heading);
        boolean left;
        boolean done=false;


        if(gyro.getDelta(destAngle, initAngle) > 0){
            left=false;
        }
        else{
            left=true;
        }


        if(left){
            if(delta > 0){
                done=true;
            }
        }
        else{
            if(delta < 0){
                done=true;
            }
        }

        //If heading is not at destination
        if (!done) {

            telemetry.addData("Angle: ", delta);
            telemetry.addData("Delta: ",delta);

            //Drive the motors so the robot turns
            if(!left){ //For red side
                fLeft.setPower(1);
                fRight.setPower(-1);
                bLeft.setPower(-0.25);
                bRight.setPower(-0.7);
            }
            else{ //For blue side
                bLeft.setPower(-1);
                bRight.setPower(1);
                fLeft.setPower(0.25);
                fRight.setPower(0.7);
            }

        }
        else {
            fLeft.setPower(0);
            fRight.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);

            last_ang = true;

            return true;
        }

        return false;
    }

    /**
     * turns to given angle without correction
     * @param distance angle destination
     * @param power drive power
     */
    public boolean simpleTurn(double distance,double power){
        telemetry.addData("angle",gyro.getRawYaw());
        if(distance<gyro.getRawYaw()) {
            telemetry.update();
            fLeft.setPower(-power);
            bLeft.setPower(-power);
            bRight.setPower(power);
            fRight.setPower(power);
        }
        else if (distance > gyro.getRawYaw()) {
            telemetry.update();
            fLeft.setPower(power);
            bLeft.setPower(power);
            bRight.setPower(-power);
            fRight.setPower(-power);
        }
        else {
            fLeft.setPower(0);
            bLeft.setPower(0);
            bRight.setPower(0);
            fRight.setPower(0);

            return true;
        }

        return false;
    }

    /**
     * Drives to a line and then drives a distance
     */
    public boolean driveColorDist(ColorSensor color, Directions dir, double dist, double ang){

        int direction;
        if (driveColor) {

            if (dir == Directions.FORWARD || dir == Directions.BACKWARD) {
                direction = dir == Directions.FORWARD ? 1 : -1;

                if (color.red() < 65 && color.blue() < 65) {
                    fLeft.setPower(0.5 * direction);
                    bLeft.setPower(0.5 * direction);
                    bRight.setPower(0.5 * direction);
                    fRight.setPower(0.5 * direction);
                } else {
                    driveColor = false;
                }
            } else {
                direction = dir == Directions.RIGHT ? -1 : 1;

                if (color.red() < 65 && color.blue() < 65) {
                    fLeft.setPower(-0.6 * direction);
                    bLeft.setPower(0.6 * direction);
                    bRight.setPower(-0.6 * direction);
                    fRight.setPower(0.6 * direction);
                } else {
                    driveColor = false;
                }
            }
        }
        else{
            if (driveStraight(dir, dist, ang)) {
                driveColor = true;
                return true;
            }
        }

        return false;
    }

    /**
     * Drives or strafes to colored line depending on direction
     * @param color Color sensor to use
     * @param dir   Direction to drive / strafe in
     * @return
     */
    public boolean driveToLine(ColorSensor color, Directions dir){

        int direction;

        if (dir == Directions.FORWARD || dir == Directions.BACKWARD) {
            direction = dir == Directions.FORWARD ? 1 : -1;

            if (color.red()<50&&color.blue()<60){
                fLeft.setPower(0.6*direction);
                bLeft.setPower(0.6*direction);
                bRight.setPower(0.6*direction);
                fRight.setPower(0.6*direction);
            }
            else{
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }
        else {
            direction = dir == Directions.RIGHT ? -1 : 1;

            if (color.red()<80&&color.blue()<80){
                fLeft.setPower(-0.6*direction);
                bLeft.setPower(0.6*direction);
                bRight.setPower(-0.6*direction);
                fRight.setPower(0.6*direction);
            }
            else {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }

        return false;
    }

    /**
     * strafe to a given range.
     * Need to call sensors distanceR and distanceL
     * @param dSensor distance sensor object
     * @param distance distance driving to
     * @param sLocal location of sensor(center,right,left)
     */
    public boolean driveRange(DistanceSensor dSensor, double distance, Positions sLocal){
        double cDistance=0;
        if(dSensor.getDistance(DistanceUnit.CM)>150){
            cDistance=150;
        }
        else{
            cDistance=dSensor.getDistance(DistanceUnit.CM);
        }
        int dir;
        if(sLocal == Positions.CENTER) {
            if(distance<cDistance){
                if(dSensor.getDistance(DistanceUnit.CM)>150){
                    cDistance=150;
                }
                else{
                    cDistance=dSensor.getDistance(DistanceUnit.CM);
                }
                telemetry.update();
                fLeft.setPower(-0.4);
                fRight.setPower(-0.4);
                bLeft.setPower(-0.4);
                bRight.setPower(-0.4);
            }
            else if(distance>cDistance) {
                if(dSensor.getDistance(DistanceUnit.CM)>150){
                    cDistance=150;
                }
                else{
                    cDistance=dSensor.getDistance(DistanceUnit.CM);
                }
                fLeft.setPower(0.4);
                fRight.setPower(0.4);
                bLeft.setPower(0.4);
                bRight.setPower(0.4);
            }
            if (distance < cDistance + 0.75 && distance > cDistance - 0.75) {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }
        else{
            if(sLocal == Positions.RIGHT){
                dir = 1;
            }
            else{
                dir = -1;
            }

            if (distance>dSensor.getDistance(DistanceUnit.CM)) {
                if(Math.abs(distance-dSensor.getDistance(DistanceUnit.CM))>20){
                    fLeft.setPower(0.7*dir);
                    fRight.setPower(-0.7*dir);
                    bLeft.setPower(-0.7*dir);
                    bRight.setPower(0.7*dir);
                }
                else{
                    fLeft.setPower(0.4*dir);
                    fRight.setPower(-0.4*dir);
                    bLeft.setPower(-0.4*dir);
                    bRight.setPower(0.4*dir);
                }
            }
            else if (distance<dSensor.getDistance(DistanceUnit.CM)) {
                if(Math.abs(distance-dSensor.getDistance(DistanceUnit.CM))>20){
                    fLeft.setPower(-0.7*dir);
                    fRight.setPower(0.7*dir);
                    bLeft.setPower(0.7*dir);
                    bRight.setPower(-0.7*dir);
                }
                else{
                    fLeft.setPower(-0.4*dir);
                    fRight.setPower(0.4*dir);
                    bLeft.setPower(0.4*dir);
                    bRight.setPower(-0.4*dir);
                }
            }
            if (distance < dSensor.getDistance(DistanceUnit.CM) + 0.75 && distance > dSensor.getDistance(DistanceUnit.CM) - 0.75) {
                fLeft.setPower(0);
                bLeft.setPower(0);
                bRight.setPower(0);
                fRight.setPower(0);

                return true;
            }
        }

        return false;
    }

    /**
     * Waits until the set time passes
     */
    public boolean delay(double time){
        if (last_time != time) {
            last_time = time;
            endTime = System.currentTimeMillis() + time;
        }
        if (System.currentTimeMillis() >= endTime) {
            last_time = 0;
            return true;
        }

        return false;
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public double getPosX() {
        return oP.positionX;
    }

    public double getPosY() {
        return oP.positionY;
    }

    public double getRawX() {
        return oP.getRawX();
    }

    public double getRawY() {
        return oP.getRawY();
    }

    public void setX(double x) {
        oP.setX(x);
    }

    public void setY(double y) {
        oP.setY(y);
    }
}