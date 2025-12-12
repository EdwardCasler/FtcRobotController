package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class GoBuildaTeleop extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    final double targetVelocity = 1125;
    ElapsedTime timer = new ElapsedTime();
    boolean launcherOn = false;
    boolean feedersSpinning;
    public void init() {
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

        telemetry.addLine("Y to turn launcher off/on");
        telemetry.addLine("A to launch");
        telemetry.addLine("X for intake turning off");
        telemetry.addLine("Right & Left bumper for intake left right");
        telemetry.addLine("Right & Left trigger for feeders");
    }
    public void loop() {
        drive();
        turnOn();
    }
    public void turnOn() {
        if (gamepad1.yWasReleased()) {
            launcherOn = !launcherOn;
            launcher.setVelocity(launcherOn ? targetVelocity : 0);
        }
        if(gamepad1.aWasReleased() && !feedersSpinning){
            leftFeeder.setPower(1);
            rightFeeder.setPower(1);
            feedersSpinning = true;
            timer.reset();
        }
        if(feedersSpinning && timer.milliseconds() > 150) {
            feedersSpinning = false;
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
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
    public void drive() {
        double denominator = Math.max(Math.abs(-gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        leftFrontDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator);
        rightFrontDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator);
        leftBackDrive.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator);
        rightBackDrive.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator);
    }
}
