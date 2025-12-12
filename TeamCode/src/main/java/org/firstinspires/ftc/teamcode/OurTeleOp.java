package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp
public class OurTeleOp extends OpMode {
    private DcMotorEx flywheel;
    private DcMotor feedRoller;
    private DcMotor leftDrive;
    private CRServo agitator;
    private DcMotor rightDrive;

    // Dashboard instance
    private FtcDashboard dashboard;

    private float flyWheelVelocity = 1300;
    private static final int bankVelocity = 1300;
    private static final int farVelocity = 1900;
    private static final int maxVelocity = 2200;

    private boolean flyWheelPowered;
    private boolean agitatorPowered;
    private boolean feedRollerPowered;

    @Override
    public void init() {
        // --- Hardware Mapping ---
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feedRoller = hardwareMap.get(DcMotor.class, "coreHex");
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        agitator = hardwareMap.get(CRServo.class, "servo");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        // --- Motor Setup ---
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedRoller.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        agitator.setDirection(DcMotor.Direction.REVERSE);

        // --- Dashboard Initialization ---
        dashboard = FtcDashboard.getInstance();

        // Driver Hub Telemetry
        telemetry.addLine("--- OurTeleOp Initialized ---");
        telemetry.addLine("Connect to the Dashboard via web browser.");
        telemetry.update();

        // Dashboard Telemetry (initial instructions)
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Instructions", "a: flywheel | b: agitator | x: feed roller");
        packet.put("Speed Control", "LB: slow | RB: medium");
        packet.put("Note", "Voltage control is active");
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        basicMovement();
        turnOnMotors();
        flyWheel();

        // --- Dashboard Telemetry in Loop ---
        TelemetryPacket packet = new TelemetryPacket();
        double voltage = getLowestVoltage();

        packet.put("Flywheel Velocity (Target)", flyWheelVelocity);
        packet.put("Flywheel Velocity (Actual)", flywheel.getVelocity());
        packet.put("Flywheel Status", flyWheelPowered ? "ON" : "OFF");
        packet.put("Agitator Status", agitatorPowered ? "ON" : "OFF");
        packet.put("Lowest Voltage", voltage + "V");

        dashboard.sendTelemetryPacket(packet);
    }

    public double getLowestVoltage() {
        double lowestValue = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor.getVoltage() < lowestValue && sensor.getVoltage() > 0.1) {
                lowestValue = sensor.getVoltage();
            }
        }
        if (lowestValue == Double.POSITIVE_INFINITY) {
            lowestValue = 14; // safe default
        }
        return lowestValue;
    }

    public void basicMovement() {
        float x = gamepad1.right_stick_x;
        float y = gamepad1.left_stick_y;
        leftDrive.setPower(y - x);
        rightDrive.setPower(y + x);
    }

    public void turnOnMotors() {
        if (gamepad1.a) {
            flyWheelPowered = !flyWheelPowered;
        }
        if (gamepad1.b) {
            agitatorPowered = !agitatorPowered;
            agitator.setPower(agitatorPowered ? 1 : 0);
        }
        if (gamepad1.x) {
            feedRollerPowered = !feedRollerPowered;
            feedRoller.setPower(feedRollerPowered ? 1 : 0);
        }
        if (gamepad1.y) {
            feedRoller.setPower(-0.5);
        } else {
            if (!feedRollerPowered) {
                feedRoller.setPower(0);
            }
        }
    }

    public void flyWheel() {
        if (flyWheelPowered) {
            double multiplier = 14.0 / getLowestVoltage();
            flywheel.setVelocity(flyWheelVelocity * multiplier);
        } else {
            flywheel.setVelocity(0);
        }
    }
}
