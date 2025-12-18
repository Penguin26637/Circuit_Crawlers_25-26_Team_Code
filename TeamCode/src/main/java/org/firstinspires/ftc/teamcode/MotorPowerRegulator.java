package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotorPowerRegulator {

    public static double TICKS_PER_REV = 112.0;
    public static double MAX_RPM_UNDER_LOAD = 1400.0;

    public static double kV = 0.0006785714285714286;
    public static double kS = 0.06;

    public static double kP = 0.0004;
    public static double kI = 0.0002;
    public static double kD = 0.00005;

    public static double integralLimit = 0.2;

    private DcMotorEx flywheel;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String motorName;
    private ElapsedTime dtTimer = new ElapsedTime();
    private double lastPosition = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private double currentRPM = 0.0;

    public static double targetRPM = 980.0;

    private FtcDashboard dashboard;

    public MotorPowerRegulator(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.motorName = motorName;

        flywheel = hardwareMap.get(DcMotorEx.class, motorName);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        lastPosition = flywheel.getCurrentPosition();
        dtTimer.reset();

        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("=== " + motorName.toUpperCase() + " FLYWHEEL INITIALIZED ===");
        telemetry.addLine("Motor: 6000 RPM no-load spec");
        telemetry.addLine("Measured: 1400 RPM at full power");
        telemetry.addLine("System is now calibrated!");
        telemetry.update();
    }

    public MotorPowerRegulator(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "shooter");
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flywheel.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return flywheel.getZeroPowerBehavior();
    }

    public void setMode(DcMotor.RunMode runMode) {
        flywheel.setMode(runMode);
        if (runMode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            lastPosition = 0;
            integral = 0;
            lastError = 0;
            dtTimer.reset();
        }
    }

    public DcMotor.RunMode getMode() {
        return flywheel.getMode();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        flywheel.setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return flywheel.getDirection();
    }

    public DcMotorEx getMotor() {
        return flywheel;
    }

    public void setTicksPerRev(double ticksPerRev) {
        TICKS_PER_REV = ticksPerRev;
    }

    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    public void setMaxRpmUnderLoad(double maxRpm) {
        MAX_RPM_UNDER_LOAD = maxRpm;
    }

    public double getMaxRpmUnderLoad() {
        return MAX_RPM_UNDER_LOAD;
    }

    public void setKv(double kv) {
        kV = kv;
    }

    public double getKv() {
        return kV;
    }

    public void setKs(double ks) {
        kS = ks;
    }

    public double getKs() {
        return kS;
    }

    public void setKp(double kp) {
        kP = kp;
    }

    public double getKp() {
        return kP;
    }

    public void setKi(double ki) {
        kI = ki;
    }

    public double getKi() {
        return kI;
    }

    public void setKd(double kd) {
        kD = kd;
    }

    public double getKd() {
        return kD;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(MAX_RPM_UNDER_LOAD, rpm));
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public boolean isAtTarget(double toleranceRPM) {
        return Math.abs(currentRPM - targetRPM) < toleranceRPM;
    }

    public void resetIntegral() {
        integral = 0.0;
    }

    public void stop() {
        flywheel.setPower(0);
        targetRPM = 0;
        integral = 0;
    }

    public void setPIDGains(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }

    public void setFeedforwardGains(double kv, double ks) {
        kV = kv;
        kS = ks;
    }

    public void setAllGains(double kv, double ks, double kp, double ki, double kd) {
        kV = kv;
        kS = ks;
        kP = kp;
        kI = ki;
        kD = kd;
    }

    public String getMotorName() {
        return motorName;
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) result = Math.min(result, voltage);
        }
        return result;
    }

    public void loop() {
        double pos = flywheel.getCurrentPosition();
        double dt = dtTimer.seconds();

        if (dt < 0.001) dt = 0.001;
        dtTimer.reset();

        lastPosition = pos;

        double velocityTicksPerSec = flywheel.getVelocity();
        currentRPM = (velocityTicksPerSec / TICKS_PER_REV) * 60.0;

        double ff = 0.0;
        if (targetRPM > 20) {
            ff = kS + kV * targetRPM;
        }

        double error = targetRPM - currentRPM;

        integral += error * dt;
        integral = Math.max(-integralLimit, Math.min(integralLimit, integral));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double pid = kP * error + kI * integral + kD * derivative;

        double output = ff + pid;
        output = Math.max(-1.0, Math.min(1.0, output));

        flywheel.setPower(output);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put(motorName + " Target RPM", targetRPM);
        packet.put(motorName + " Actual RPM", currentRPM);
        packet.put(motorName + " Error RPM", error);
        packet.put(motorName + " Error %", targetRPM != 0 ? (error / targetRPM) * 100.0 : 0.0);

        packet.put(motorName + " Total Power", output);
        packet.put(motorName + " FF Power", ff);
        packet.put(motorName + " PID Power", pid);
        packet.put(motorName + " Integral", integral);

        packet.put("TICKS_PER_REV", TICKS_PER_REV);
        packet.put("MAX_RPM_UNDER_LOAD", MAX_RPM_UNDER_LOAD);
        packet.put("kV", kV);
        packet.put("kS", kS);
        packet.put("kP", kP);
        packet.put("kI", kI);
        packet.put("kD", kD);

        double batteryVoltage = getBatteryVoltage();

        packet.put(motorName + " Saturated?", Math.abs(output) >= 0.99);
        packet.put(motorName + " FF % of Total", output != 0 ? (ff / output) * 100.0 : 0.0);
        packet.put("battery_Voltage", batteryVoltage);
        packet.put(motorName + " Encoder Ticks", pos);
        packet.put(motorName + " Run Mode", flywheel.getMode().toString());
        packet.put(motorName + " Zero Power Behavior", flywheel.getZeroPowerBehavior().toString());

        dashboard.sendTelemetryPacket(packet);

        targetRPM = Math.max(0, Math.min(MAX_RPM_UNDER_LOAD, targetRPM));

        telemetry.addData("🎯 " + motorName + " Target", "%.0f RPM", targetRPM);
        telemetry.addData("⚡ " + motorName + " Actual", "%.0f RPM (%.1f%%)",
                currentRPM, targetRPM != 0 ? (currentRPM / targetRPM) * 100 : 0);
        telemetry.addData("🔋 " + motorName + " Power", "%.2f", output);
        telemetry.addData("📊 " + motorName + " Error", "%.0f RPM", error);
        telemetry.addLine();
        telemetry.addData("Max Under Load", "%.0f RPM", MAX_RPM_UNDER_LOAD);

        if (Math.abs(output) >= 0.99) {
            telemetry.addLine("⚠️ " + motorName + " SATURATED");
        }

        telemetry.update();
    }
}
