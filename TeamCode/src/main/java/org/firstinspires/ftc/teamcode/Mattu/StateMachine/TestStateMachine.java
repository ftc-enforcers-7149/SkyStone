package org.firstinspires.ftc.teamcode.Mattu.StateMachine;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

public class TestStateMachine extends OpMode {

    //State Machine
    CustomStateMachine stateMachine;
    LookForObstacle lookForObstacle;
    AvoidObstacle avoidObstacle;

    //Hardware
    DistanceSensor dLeft, dRight, dFront;

    public void init() {
        //Hardware map
        dLeft = hardwareMap.get(DistanceSensor.class, "distanceL");
        dRight = hardwareMap.get(DistanceSensor.class, "distanceR");
        dFront = hardwareMap.get(DistanceSensor.class, "distanceC");

        stateMachine = new CustomStateMachine();
        lookForObstacle = new LookForObstacle(dLeft, dRight, dFront);
        avoidObstacle = new AvoidObstacle(dLeft, dRight, dFront);

        stateMachine.addTransition(new CustomStateTransition(lookForObstacle, Events.DETECTED_OBSTACLE, avoidObstacle));
        stateMachine.addTransition(new CustomStateTransition(avoidObstacle, Events.AVOIDED_OBSTACLE, lookForObstacle));

        stateMachine.start(lookForObstacle);
    }

    public void loop() {

    }

    public void stop() {

    }
}