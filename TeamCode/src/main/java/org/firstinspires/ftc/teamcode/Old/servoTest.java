package org.firstinspires.ftc.teamcode.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends LinearOpMode {
    private Servo flap;
    private float positionPerDegree = 1f / 270f;
    public void runOpMode() {
        flap = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.yWasPressed()) {
                flap.setPosition(positionPerDegree * 140);
            }
            if(gamepad1.aWasPressed()) {
                flap.setPosition(positionPerDegree * 70);
            }
        }
    }
}
