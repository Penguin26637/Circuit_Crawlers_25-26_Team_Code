package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Dual V3 Smoothed Detection Dashboard", group = "Testing")
@Config
public class DualColorSensorArtifact extends LinearOpMode {

    // Hue thresholds (editable on dashboard)
    public static float greenMinHue = 90;
    public static float greenMaxHue = 150;
    public static float purpleMinHue = 230;
    public static float purpleMaxHue = 300;

    // Number of readings to average
    public static int smoothingSamples = 5;

    RevColorSensorV3 cs1;
    RevColorSensorV3 cs2;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // Initialize sensors
        cs1 = hardwareMap.get(RevColorSensorV3.class, "cs");
        cs2 = hardwareMap.get(RevColorSensorV3.class, "cs2");

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            // Get smoothed hue readings
            float cs1Hue = getAverageHue(cs1, smoothingSamples);
            float cs2Hue = getAverageHue(cs2, smoothingSamples);

            // Average between both sensors
            float avgHue = (cs1Hue + cs2Hue) / 2.0f;

            // Robust color detection
            String detectedColor = "Unknown";
            if (avgHue >= greenMinHue && avgHue <= greenMaxHue) {
                detectedColor = "Green";
            } else if (avgHue >= purpleMinHue && avgHue <= purpleMaxHue) {
                detectedColor = "Purple";
            }

            // Dashboard telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("CS1 Hue", cs1Hue);
            packet.put("CS2 Hue", cs2Hue);
            packet.put("Average Hue", avgHue);
            packet.put("Detected Color", detectedColor);
            dashboard.sendTelemetryPacket(packet);

            // Driver Station telemetry
            telemetry.addData("CS1 Hue", cs1Hue);
            telemetry.addData("CS2 Hue", cs2Hue);
            telemetry.addData("Avg Hue", avgHue);
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
        }
    }

    // Returns average hue over multiple readings for smoothing
    private float getAverageHue(RevColorSensorV3 sensor, int samples) {
        float sumHue = 0;
        for (int i = 0; i < samples; i++) {
            float r = sensor.red();
            float g = sensor.green();
            float b = sensor.blue();
            int colorInt = Color.rgb((int) r, (int) g, (int) b);
            float[] hsv = new float[3];
            Color.colorToHSV(colorInt, hsv);
            sumHue += hsv[0];
            sleep(5); // short delay between readings
        }
        return sumHue / samples;
    }
}
