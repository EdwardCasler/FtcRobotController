package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class GoBuildaAuto extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx leftFeeder = null;
    private DcMotorEx rightFeeder = null;
    final double targetVelocity = 1125;
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        leftFeeder = hardwareMap.get(DcMotorEx.class, "left_feeder");
        rightFeeder = hardwareMap.get(DcMotorEx.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        launcher.setVelocity(targetVelocity);
        sleep(5000);
        leftFeeder.setPower(1);
        rightFeeder.setPower(1);
        sleep(5000);
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
        drive(-1, 0.2f, 0);
        sleep(1000);
        drive(0, 0 ,0);
    }
    public void drive(float forward, float strafe, float rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }
}
