package org.firstinspires.ftc.teamcode.PreComp2Files;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name="Battery Monitor", group="Utility")
public class BatteryMonitor extends LinearOpMode {

    // Declare a variable to hold the FtcDashboard instance
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        // Initialize the FtcDashboard instance
        dashboard = FtcDashboard.getInstance();

        // Get the voltage sensor from the hardware map
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (opModeIsActive()) {
            // Create a new TelemetryPacket for this iteration
            TelemetryPacket packet = new TelemetryPacket();

            // Get battery voltage
            double voltage = batteryVoltageSensor.getVoltage();

            // Calculate battery percentage (12V battery: 12.0V = 100%, 11.0V = 0%)
            // Adjust these values based on your battery type
            double maxVoltage = 13.0;  // Fully charged voltage
            double minVoltage = 11.0;  // Minimum safe voltage

            double batteryPercentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100;

            // Clamp percentage between 0 and 100
            batteryPercentage = Math.max(0, Math.min(100, batteryPercentage));

            // Send to telemetry (Driver Station)
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            telemetry.addData("Battery Percentage", "%.1f%%", batteryPercentage);
            telemetry.update();

            // Send to FTC Dashboard
            packet.put("Battery Voltage", voltage);
            packet.put("Battery Percentage", batteryPercentage);
            dashboard.sendTelemetryPacket(packet);

            sleep(100);  // Update every 100ms
        }
    }
}