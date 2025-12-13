package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class GoBuildaTeleop extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    final double targetVelocity = 1225;
    ElapsedTime timer = new ElapsedTime();
    boolean feedersSpinning;
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("A to launch");
        telemetry.addLine("X for intake turning off");
        telemetry.addLine("Right & Left bumper for intake left right");
        telemetry.addLine("Right & Left trigger for feeders");

        waitForStart();
        while (opModeIsActive()) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            turnOn();
        }
    }
    float timesShot = 0;
    public void turnOn() {
        if(gamepad1.aWasReleased() && !feedersSpinning) {
            drive(0, 0, 0);
            launcher.setVelocity(targetVelocity);
            timer.reset();
            while (timesShot <= 3 && opModeIsActive()) {
                if (launcher.getVelocity() > targetVelocity - 25 && launcher.getVelocity() < targetVelocity + 25 && !feedersSpinning && timer.milliseconds() > 1000) {
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    timesShot += 1;
                    feedersSpinning = true;
                    timer.reset();
                }
                if (timer.milliseconds() > 150) {
                    feedersSpinning = false;
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                }
            }
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            launcher.setVelocity(0);
            feedersSpinning = false;
            timesShot = 0;
        }

        if(gamepad1.xWasReleased()){
            intake.setPower(0);
        }
        if(gamepad1.leftBumperWasPressed()) {
            intake.setPower(1);
            intake.setPower(1);
        }
        if(gamepad1.rightBumperWasPressed()) {
            intake.setPower(-1);
            intake.setPower(-1);
        }
    }
    void drive(float forward, float strafe, float rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }
}
