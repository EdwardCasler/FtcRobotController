package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class OurRedAuto extends LinearOpMode {
    private DcMotorEx flywheel;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private float positionPerDegree = 1 / (270 / 2);
    private Servo flap;
    private float upPos = positionPerDegree * 140;
    private float downPos = positionPerDegree * 70;
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flap = hardwareMap.get(Servo.class, "servo");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        flywheel.setVelocity(1700);
        sleep(5000);
        flap.setPosition(upPos);
        sleep(4000);
        leftDrive.setPower(-1);
        rightDrive.setPower(1);
        sleep(100);
        leftDrive.setPower(-1);
        rightDrive.setPower(-1);
        sleep(1000);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        flywheel.setVelocity(0);
    }
}