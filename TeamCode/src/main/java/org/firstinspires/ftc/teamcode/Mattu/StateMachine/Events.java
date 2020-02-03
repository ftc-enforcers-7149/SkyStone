package org.firstinspires.ftc.teamcode.Mattu.StateMachine;

import org.firstinspires.ftc.robotcore.external.Event;

public enum Events implements Event {
    DETECTED_OBSTACLE,
    AVOIDED_OBSTACLE;

    public String getName() {
        return "Events";
    }
}
