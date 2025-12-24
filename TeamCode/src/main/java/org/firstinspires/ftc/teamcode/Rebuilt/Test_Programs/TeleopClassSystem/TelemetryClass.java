package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TelemetryClass - Handles telemetry display
 * References to other classes are public
 */
public class TelemetryClass {

    // Public variables
    public Telemetry telemetry;
    public FtcDashboard dashboard;
    public HardwareMap hardwareMap;

    // References to other classes - Public - Set from main teleop
    public DriveControlClass drive;
    public IntakeClass intake;
    public ShooterClass shooter;
    public OdometryClass odometry;

    // Status - Public
    public boolean enabled = false;
    public boolean dashboardEnabled = false;
    public boolean initialized = false;

    public TelemetryClass(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        try {
            dashboard = FtcDashboard.getInstance();
            initialized = true;
        } catch (Exception e) {
            initialized = false;
            telemetry.addData("Telemetry Error", e.getMessage());
        }
    }

    public void update(boolean enabled, boolean dashboardEnabled) {
        this.enabled = enabled;
        this.dashboardEnabled = dashboardEnabled;

        if (!enabled || !initialized) return;

        sendDriverStationTelemetry();

        if (dashboardEnabled) {
            sendDashboardTelemetry();
        }

        telemetry.update();
    }

    public void sendDriverStationTelemetry() {
        telemetry.addLine("=== ROBOT STATUS ===");
        telemetry.addData("Battery (V)", getBatteryVoltage());
        telemetry.addLine();

        if (drive != null && drive.getInitialized()) {
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Mode", drive.useFieldCentric ? "Field Centric" : "Robot Centric");

            if (drive.useFieldCentric) telemetry.addData("Using", drive.useImuForFieldCentric ? "IMU" : "Odometry");
            telemetry.addData("Speed", "%.2f", drive.nerf);
            telemetry.addData("Wheel Brake", drive.useWheelBrake ? "ON" : "OFF");
            telemetry.addData("Back PID", drive.useBackMotorPid ? "ON" : "OFF");

            if (drive.useBackMotorPid) {
                telemetry.addData("BL Target", "%.0f RPM", drive.getBackLeftTargetRPM());
                telemetry.addData("BL Actual", "%.0f RPM", drive.getBackLeftCurrentRPM());
                telemetry.addData("BR Target", "%.0f RPM", drive.getBackRightTargetRPM());
                telemetry.addData("BR Actual", "%.0f RPM", drive.getBackRightCurrentRPM());
            }
            telemetry.addLine();
        }

        if (intake != null && intake.getInitialized()) {
            telemetry.addLine("=== INTAKE ===");
            if (intake.getIntake1Initialized()) {
                telemetry.addData("Intake 1 Target", "%.2f", intake.targetPosition);
                telemetry.addData("Intake 1 Actual", "%.2f", intake.getIntake1Value());
            }
            if (intake.getIntake2Initialized()) {
                telemetry.addData("Intake 2 Target", "%.2f", intake.targetPosition);
                telemetry.addData("Intake 2 Actual", "%.2f", intake.getIntake2Value());
            }
            telemetry.addLine();
        }

        if (shooter != null && shooter.getShooterInitialized()) {
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Target RPM", "%.0f", shooter.shooterTargetRPM);
            telemetry.addData("Actual RPM", "%.0f", shooter.getShooterCurrentRPM());
            telemetry.addData("At Target", shooter.getShooterAtTarget(50) ? "✓" : "✗");

            if (shooter.getHinge1Initialized()) {
                telemetry.addData("Hinge Target", "%.2f", shooter.hinge1TargetPosition);
                telemetry.addData("Hinge Actual", "%.2f", shooter.getHinge1Position());
            }
            if (shooter.getHinge2Initialized()) {
                telemetry.addData("Hinge Target", "%.2f", shooter.hinge2TargetPosition);
                telemetry.addData("Hinge Actual", "%.2f", shooter.getHinge2Position());
            }

            if (shooter.getMagazineInitialized()) {
                telemetry.addData("Magazine Power", "%.2f", shooter.magazineTargetPower);
            }
            telemetry.addLine();
        }

        if (odometry != null && odometry.getInitialized()) {
            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X", "%.2f in", odometry.xPos);
            telemetry.addData("Y", "%.2f in", odometry.yPos);
            telemetry.addData("Heading", "%.2f°", odometry.getHeadingDegrees());
            telemetry.addLine();
        }
    }

    public void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Battery (V)", getBatteryVoltage());

        if (drive != null && drive.getInitialized()) {
            packet.put("Field Centric", drive.useFieldCentric);
            packet.put("Use IMU", drive.useImuForFieldCentric);
            packet.put("Speed Multiplier", drive.nerf);
            packet.put("Wheel Brake", drive.useWheelBrake);
            packet.put("Back Motor PID", drive.useBackMotorPid);

            if (drive.useBackMotorPid) {
                packet.put("Back Left Target RPM", drive.getBackLeftTargetRPM());
                packet.put("Back Left Actual RPM", drive.getBackLeftCurrentRPM());
                packet.put("Back Right Target RPM", drive.getBackRightTargetRPM());
                packet.put("Back Right Actual RPM", drive.getBackRightCurrentRPM());
            }
        }

        if (intake != null && intake.getInitialized()) {
            if (intake.getIntake1Initialized()) {
                packet.put("Intake 1 Target", intake.targetPosition);
                packet.put("Intake 1 Actual", intake.getIntake1Value());
            }
            if (intake.getIntake2Initialized()) {
                packet.put("Intake 2 Target", intake.targetPosition);
                packet.put("Intake 2 Actual", intake.getIntake2Value());
            }
        }

        if (shooter != null && shooter.getShooterInitialized()) {
            packet.put("Shooter Target RPM", shooter.shooterTargetRPM);
            packet.put("Shooter Actual RPM", shooter.getShooterCurrentRPM());
            packet.put("Shooter At Target", shooter.getShooterAtTarget(50));

            if (shooter.getHinge1Initialized()) {
                packet.put("Hinge 1 Target", shooter.hinge1TargetPosition);
                packet.put("Hinge 1 Actual", shooter.getHinge1Position());
            }
            if (shooter.getHinge2Initialized()) {
                packet.put("Hinge 2 Target", shooter.hinge2TargetPosition);
                packet.put("Hinge 2 Actual", shooter.getHinge2Position());
            }

            if (shooter.getMagazineInitialized()) {
                packet.put("Magazine Power", shooter.magazineTargetPower);
            }
        }

        if (odometry != null && odometry.getInitialized()) {
            packet.put("X (in)", odometry.xPos);
            packet.put("Y (in)", odometry.yPos);
            packet.put("Heading (deg)", odometry.getHeadingDegrees());

            Canvas canvas = packet.fieldOverlay();
            drawRobot(canvas, odometry.xPos, odometry.yPos, odometry.heading);
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void drawRobot(Canvas canvas, double x, double y, double heading) {
        // Grid
        canvas.setStroke("#404040");
        for (int i = -72; i <= 72; i += 24) {
            canvas.strokeLine(i, -72, i, 72);
            canvas.strokeLine(-72, i, 72, i);
        }

        // Border
        canvas.setStroke("#FFFFFF");
        canvas.strokeRect(-72, -72, 144, 144);

        // Origin
        canvas.setStroke("#FFFF00");
        canvas.strokeLine(-10, 0, 10, 0);
        canvas.strokeLine(0, -10, 0, 10);

        // Robot
        double size = 18;
        canvas.setStroke("#3FBAFF");
        canvas.setFill("#3FBAFF");
        canvas.fillRect(x - size / 2, y - size / 2, size, size);

        // Heading line
        double len = 12;
        double hx = x + len * Math.cos(heading);
        double hy = y + len * Math.sin(heading);
        canvas.setStroke("#FF0000");
        canvas.setStrokeWidth(3);
        canvas.strokeLine(x, y, hx, hy);

        // Center
        canvas.setStroke("#00FF00");
        canvas.fillCircle(x, y, 3);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    // ==================== GETTERS ====================
    public boolean getInitialized() { return initialized; }
    public boolean getEnabled() { return enabled; }
    public boolean getDashboardEnabled() { return dashboardEnabled; }
}
