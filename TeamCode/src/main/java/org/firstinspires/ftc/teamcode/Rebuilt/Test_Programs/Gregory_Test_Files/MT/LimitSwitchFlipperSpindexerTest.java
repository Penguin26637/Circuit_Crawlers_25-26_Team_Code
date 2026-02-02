package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "LimitSwitch + Flipper + Spindexer Test", group = "Test")
public class LimitSwitchFlipperSpindexerTest extends LinearOpMode {

    private MultipleTelemetry multitelemetry;

    private DigitalChannel limitSwitch;
    private ServoClassMT Flipper, Spindexer;

    @Override
    public void runOpMode() {

        multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        try {
            limitSwitch = hardwareMap.get(DigitalChannel.class, "touch");
            limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            multitelemetry.addData("Error", "Limit switch not found!");
        }

        Flipper = safeInitServo("Flipper", ServoClassMT.ServoType.STANDARD_SERVO);
        Spindexer = safeInitServo("Spindexer", ServoClassMT.ServoType.CONTINUOUS_SERVO);

        multitelemetry.addData("Status", "Ready! Press Play");
        multitelemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean pressed = isLimitSwitchPressed();

            // If limit switch pressed, stop spindexer and move flipper up
            if (pressed) {
                safeGoToPosition(Flipper, 0.2);
                Spindexer.stop();
            } else {
                // Otherwise, move spindexer forward slowly
                Spindexer.goToPosition(1);
                safeGoToPosition(Flipper, 0);
            }

            // Update telemetry
            updateTelemetry("Limit Switch", pressed);

            sleep(100);
        }
    }

    private ServoClassMT safeInitServo(String name, ServoClassMT.ServoType type) {
        try {
            ServoClassMT servo = new ServoClassMT(name, type);
            servo.init(hardwareMap);
            return servo;
        } catch (Exception e) {
            multitelemetry.addData("Error", name + " not found!");
            multitelemetry.update();
            return null;
        }
    }

    private void safeGoToPosition(ServoClassMT servo, double position) {
        if (servo != null) servo.goToPosition(position);
    }

    private boolean isLimitSwitchPressed() {
        if (limitSwitch == null) return false;
        return limitSwitch.getState();
    }

    private void updateTelemetry(String name, boolean pressed) {
        multitelemetry.addData(name + " Pressed", pressed);
        multitelemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put(name + " Pressed", pressed);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
