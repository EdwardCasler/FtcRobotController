package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    final double minVelocity = 1075;

    boolean launcherOn = false;
    boolean intakeOn = false;
    boolean intakeIn = true;
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Y to turn launcher off/on");
        telemetry.addLine("A to turn intake off/on");
        telemetry.addLine("X to turn reverse intake direction");
    }
    public void loop() {
        drive();
        turnOn();
    }
    public void turnOn() {
        if (gamepad1.y) {
            launcherOn = !launcherOn;
            launcher.setVelocity(launcherOn ? targetVelocity : 0);
        } else if(gamepad1.a){
            intakeOn = !intakeOn;
            intake.setPower(intakeOn ? 1 : 0);
        } else if(gamepad1.x){
            intakeIn = !intakeIn;
            intake.setDirection(intakeIn ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        }
    }
    public void drive() {
        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        leftFrontDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator);
        rightFrontDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator);
        leftBackDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / denominator);
        rightBackDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / denominator);
    }
}
