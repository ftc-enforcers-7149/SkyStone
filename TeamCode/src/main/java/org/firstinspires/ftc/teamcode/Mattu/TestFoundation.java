package org.firstinspires.ftc.teamcode.Mattu;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ParentInit;

@TeleOp(name = "Test Foundation")
public class TestFoundation extends ParentInit {
    public void init() {
        super.init();
    }

    public void init_loop() {
        super.init_loop();
    }

    public void loop() {
        if (gamepad1.a) {
            foundation.lHalf();
        }
        else if (gamepad1.b) {
            foundation.lDown();
        }
        else if (gamepad1.x) {
            foundation.lUp();
        }
    }
}