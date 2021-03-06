package org.firstinspires.ftc.teamcode.KrishnaSaysKilljoysNeverDie.Handling;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CollisionAvoidance.MovementDetectionClass;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrainV1;
import org.firstinspires.ftc.teamcode.Subsystems.Gyroscope;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryEncoder;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.OdometryPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry.SamplePath;

public class HandlingSystem {


    //Objects
    OdometryPosition odometryPosition;
    OdometryEncoder odometryEncoder;
    MovementDetectionClass movementDetectionClass;
    SamplePath sPath;
    Gyroscope gyroscope;
    DriveTrainV1 driveTrainV1;



    //Movement variables
    boolean lMoving, rMoving, cMoving;


    //Encoder variables
    double posX, posY;
    double offTrackR, displacementX, displacementY;
    boolean offTrack;

    //Path vars
    String path;
    int step;
    OdometryPosition.Direction direction;




    //TODO: FIX DIS STUFF. MAKE IT MOTORS.

    public HandlingSystem(HardwareMap hardwareMap, SamplePath samplePath, Gyroscope gyroscope, String encX, String encY, String imumap, double posX, double posY, String distC, String distR, String distL) {

        this.gyroscope = gyroscope;
        odometryEncoder = new OdometryEncoder(hardwareMap, encX, encY, imumap, posX, posY, gyroscope);
        //odometryPosition = new OdometryPosition(hardwareMap, encX, encY, posX, posY, gyroscope);

        odometryEncoder.startOdometry();

        sPath = samplePath;

    }

    public void handle() {

        updateAll();


        //Sensor logic
        if(cMoving && !lMoving && !rMoving) {
            //TODO: insert logic here
        }
        else if(!cMoving && lMoving && !rMoving) {
            //TODO: insert logic here
        }
        else if(!cMoving && !lMoving && rMoving) {
            //TODO: insert logic here
        }


        //Encoder logic
        if(offTrack) {

            if(step == 1) {

                if(displacementX > 0.1) {}

                if(displacementY > 0.1) {}

                //TODO: INSERT LOGIC HERE
            }
            else if(step == 2) {
                //TODO: INSERT LOGIC HERE
            }

        }



    }

    public void updateAll() {


        //Update path
        path = sPath.getPath();
        step = sPath.getStep();
        direction = sPath.getDirection();


        //Update dist sensors
        movementDetectionClass.update();



        //Update encoders
        odometryPosition.updatePosition(direction);

        offTrackR = odometryEncoder.OnTrack(path, step, direction)[0];
        displacementX = odometryEncoder.OnTrack(path, step, direction)[1];
        displacementY = odometryEncoder.OnTrack(path, step, direction)[2];

        if(offTrackR == 1) {
            offTrack = true;
        }
        else {
            offTrack = false;
        }


    }


    String EasterEgg() {

        String encode = "sghr hr zm dzrsdq dff";
        char[] decodeArray = encode.toCharArray();

        for(int i = 0; i < encode.length(); i++) {
            decodeArray[i]++;
        }

        return decodeArray.toString() + " - Krishna Bansal :)";
    }


}
