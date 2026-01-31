package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * OdometryClass - Handles odometry calculations
 * All variables are public and controlled from main teleop
 */
public class OdometryClassGreg {

    // Hardware - Public
    public DcMotor odoLeft, odoRight, odoPerp;
    public Telemetry telemetry;
    public MultipleTelemetry multitelemetry;

    // Position variables - Public
    public double xPos = 0;
    public double yPos = 0;
    public double heading = 0;

    // Previous encoder positions - Public
    public int prevLeft = 0;
    public int prevRight = 0;
    public int prevPerp = 0;

    // Constants - Public - Set these from main teleop
    public double ticksPerInch = 337.2;
    public double trackWidth = 13.5;
    public double perpOffset = 8;

    // Status - Public
    public boolean enabled = false;
    public boolean initialized = false;

    /**
     * Constructor - just initializes hardware
     */
    public OdometryClassGreg(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        try {
            odoLeft = hardwareMap.get(DcMotor.class, "ol");
            odoRight = hardwareMap.get(DcMotor.class, "or");
            odoPerp = hardwareMap.get(DcMotor.class, "perp");

            odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            try { Thread.sleep(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }

            odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            odoLeft.setPower(0);
            odoRight.setPower(0);
            odoPerp.setPower(0);

            prevLeft = odoLeft.getCurrentPosition();
            prevRight = odoRight.getCurrentPosition();
            prevPerp = odoPerp.getCurrentPosition();

            initialized = true;

        } catch (Exception e) {
            initialized = false;
            multitelemetry.addData("Odometry Error", e.getMessage());
        }
    }

    /**
     * Main update - calculates position using public constants
     */
    public void update(boolean enabled) {
        this.enabled = enabled;

        if (!enabled || !initialized) {
            return;
        }

        // Read current encoder positions
        int leftPos = -1 * odoLeft.getCurrentPosition();
        int rightPos = odoRight.getCurrentPosition();
        int perpPos = odoPerp.getCurrentPosition();

        // Calculate deltas
        int deltaLeft = leftPos - prevLeft;
        int deltaRight = rightPos - prevRight;
        int deltaPerp = perpPos - prevPerp;

        prevLeft = leftPos;
        prevRight = rightPos;
        prevPerp = perpPos;

        // Convert to inches
        double dLeft = deltaLeft / ticksPerInch;
        double dRight = deltaRight / ticksPerInch;
        double dPerp = deltaPerp / ticksPerInch;

        // Calculate changes
        double dHeading = (dRight - dLeft) / trackWidth;
        double dForward = (dLeft + dRight) / 2.0;
        double dSide = dPerp - (dHeading * perpOffset);

        // Update position
        double sinH = Math.sin(heading);
        double cosH = Math.cos(heading);

        xPos += dForward * cosH - dSide * sinH;
        yPos += dForward * sinH + dSide * cosH;
        heading += dHeading;
    }

    /**
     * Reset position - call from main teleop
     */
    public void resetPosition() {
        xPos = 0;
        yPos = 0;
        heading = 0;

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        try { Thread.sleep(100); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoPerp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeft = odoLeft.getCurrentPosition();
        prevRight = odoRight.getCurrentPosition();
        prevPerp = odoPerp.getCurrentPosition();
    }

    // ==================== GETTERS ====================

    public boolean getInitialized() { return initialized; }
    public boolean getEnabled() { return enabled; }
    public double getHeadingDegrees() { return Math.toDegrees(heading); }
    public int getLeftEncoder() { return odoLeft != null ? odoLeft.getCurrentPosition() : 0; }
    public int getRightEncoder() { return odoRight != null ? odoRight.getCurrentPosition() : 0; }
    public int getPerpEncoder() { return odoPerp != null ? odoPerp.getCurrentPosition() : 0; }
}