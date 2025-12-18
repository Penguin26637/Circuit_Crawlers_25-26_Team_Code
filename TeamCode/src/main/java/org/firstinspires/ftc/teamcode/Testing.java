package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Testing", group="Examples")
public class Testing extends OpMode {

    // Multiple instances of the same class with different motor names!
    private MotorPowerRegulator Backright;
    private MotorPowerRegulator Backleft;
    private MotorPowerRegulator flywheel;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;

    // Move nerf to instance variable
    public static double nerf = 0.75;

    @Override
    public void init() {
        // ========== Initialize hardware ==========
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");

        // Initialize MotorPowerRegulator instances
        Backright = new MotorPowerRegulator(hardwareMap, telemetry, "backr");
        Backleft = new MotorPowerRegulator(hardwareMap, telemetry, "backl");
        flywheel = new MotorPowerRegulator(hardwareMap, telemetry, "flywheel");

        // Configure each one independently
        Backright.setTargetRPM(1000);
        Backright.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        Backleft.setTargetRPM(800);
        Backleft.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        flywheel.setTargetRPM(1200);
        flywheel.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

        telemetry.addLine("All motors initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        Backright.stop();
        Backleft.stop();
        flywheel.stop();

        // Stop front motors too
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
    }
}