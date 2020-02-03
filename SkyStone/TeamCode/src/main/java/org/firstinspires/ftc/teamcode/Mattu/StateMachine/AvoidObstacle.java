package org.firstinspires.ftc.teamcode.Mattu.StateMachine;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Event;
import org.firstinspires.ftc.robotcore.external.State;

public class AvoidObstacle implements State {

    DistanceSensor left, right, front;

    public AvoidObstacle(DistanceSensor left, DistanceSensor right, DistanceSensor front) {
        this.left = left;
        this.right = right;
        this.front = front;
    }

    public void onEnter(Event event) {

    }

    public void onExit(Event event) {

    }
}
