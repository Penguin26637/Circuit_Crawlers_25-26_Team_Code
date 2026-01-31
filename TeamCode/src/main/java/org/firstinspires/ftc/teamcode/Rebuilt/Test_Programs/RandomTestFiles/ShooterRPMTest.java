package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.RandomTestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter RPM Full Power Test", group = "Test")
public class ShooterRPMTest extends LinearOpMode {

    private DcMotorEx shooter;
    private FtcDashboard dashboard;

    // CHANGE THIS FOR YOUR MOTOR
    private static final double TICKS_PER_REV = 112; // REV HD Hex 435 RPM motor example

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();

        try {
            shooter = hardwareMap.get(DcMotorEx.class, "s");
            shooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Shooter", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Shooter Error", e.getMessage());
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        shooter.setPower(1.0);

        while (opModeIsActive()) {

            double velocityTicksPerSec = shooter.getVelocity();  // ticks/s
            double rpm = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

            // DS
            telemetry.addData("Power", shooter.getPower());
            telemetry.addData("RPM", rpm);
            telemetry.update();

            // Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Power", shooter.getPower());
            packet.put("RPM", rpm);
            dashboard.sendTelemetryPacket(packet);

            sleep(50);
        }

        shooter.setPower(0);
    }
}
