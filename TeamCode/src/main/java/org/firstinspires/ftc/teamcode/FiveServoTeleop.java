package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FiveServoTeleop extends LinearOpMode {

    private Servo servo1, servo2, servo3, servo4, servo5;


    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            setAllServos(gamepad1.left_stick_x);

            telemetry.addData("S1", servo1.getPosition());
            telemetry.addData("S2", servo2.getPosition());
            telemetry.addData("S3", servo3.getPosition());
            telemetry.addData("S4", servo4.getPosition());
            telemetry.addData("S5", servo5.getPosition());
            telemetry.update();
        }
    }

    private void setAllServos(double pos) {
        servo1.setPosition(pos);
        servo2.setPosition(pos);
        servo3.setPosition(pos);
        servo4.setPosition(pos);
        servo5.setPosition(pos);
    }
}
