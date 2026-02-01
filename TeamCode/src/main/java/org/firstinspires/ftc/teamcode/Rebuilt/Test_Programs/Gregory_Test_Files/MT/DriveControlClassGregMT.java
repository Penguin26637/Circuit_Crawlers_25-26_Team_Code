package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

/**
 * DriveControlClass - Handles drive motor logic
 * All variables are public and controlled from main teleop
 */
public class DriveControlClassGregMT {

    // Hardware - Public
    public DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public IMU imu;
    public Telemetry telemetry;
    public MultipleTelemetry multitelemetry;
    public MotorPowerRegulator_New backLeftController;
    public MotorPowerRegulator_New backRightController;

    // Control variables - Public - Set these from main teleop
    public double nerf = 0.75;
    public boolean useFieldCentric = false;
    public boolean useBackMotorPid = false;
    public boolean useWheelBrake = false;
    public boolean useImuForFieldCentric = true;  // true = IMU, false = odometry

    // Wheel brake variables - Public
    public int wheelBrakeTargetFL, wheelBrakeTargetFR, wheelBrakeTargetBL, wheelBrakeTargetBR;
    public double wheelBrakeKP = 0.01;
    public double wheelBrakeMaxPower = 0.2;
    public int wheelBrakeMaxError = 100;

    // Constants - Public
    public double DRIVE_TICKS_PER_REV = 537.7;

    // Component initialization status - Public
    public boolean initialized = false;
    public boolean driveMotorsInitialized = false;
    public boolean imuInitialized = false;
    public boolean backPidInitialized = false;

    // Enabled flag - Public
    public boolean enabled = false;


    /**
     * Constructor - initializes components independently
     */
    public DriveControlClassGregMT(HardwareMap hardwareMap, Telemetry telemetry, boolean initDriveMotors, boolean initImu, boolean initBackPid) {
        this.telemetry = telemetry;
        this.multitelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize drive motors
        if (initDriveMotors) {
            try {
                frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
                frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
                backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
                backRightDrive = hardwareMap.get(DcMotor.class, "backr");

                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.FORWARD);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                driveMotorsInitialized = true;
            } catch (Exception e) {
                driveMotorsInitialized = false;
                multitelemetry.addData("Drive Motors Error", e.getMessage());
            }
        }

        // Initialize IMU
        if (initImu) {
            try {
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                imu.initialize(parameters);
                imuInitialized = true;
            } catch (Exception e) {
                imuInitialized = false;
                multitelemetry.addData("IMU Error", e.getMessage());
            }
        }

        // Initialize back motor PID controllers
        if (initBackPid && driveMotorsInitialized) {
            try {
                backLeftController = new MotorPowerRegulator_New(hardwareMap, telemetry, "backl");
                backRightController = new MotorPowerRegulator_New(hardwareMap, telemetry, "backr");

                backLeftController.setTicksPerRev(DRIVE_TICKS_PER_REV);
                backLeftController.setMaxRpmUnderLoad(300.0);
                backLeftController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

                backRightController.setTicksPerRev(DRIVE_TICKS_PER_REV);
                backRightController.setMaxRpmUnderLoad(300.0);
                backRightController.setAllGains(0.00068, 0.06, 0.0004, 0.0002, 0.00005);

                backPidInitialized = true;
            } catch (Exception e) {
                backPidInitialized = false;
                multitelemetry.addData("Back PID Error", e.getMessage());
            }
        }

        // Overall initialization status
        initialized = driveMotorsInitialized || imuInitialized || backPidInitialized;
    }

    /**
     * Main update - reads public variables and applies drive logic
     * @param enabled Whether to run
     * @param forward Forward/backward input
     * @param strafe Left/right input
     * @param turn Turn input
     * @param odometryHeading Heading from odometry (used if useImuForFieldCentric = false)
     */
    public void update(boolean enabled, double forward, double strafe, double turn, double odometryHeading) {
        this.enabled = enabled;

        if (!enabled || !initialized || !driveMotorsInitialized) {
            stopAllMotors();
            return;
        }

        if (useWheelBrake) {
            applyWheelBrake();
        } else if (useFieldCentric) {
            // Get heading from IMU or odometry based on flag
            double heading = 0;
            if (useImuForFieldCentric && imuInitialized) {
                heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            } else {
                heading = odometryHeading;
            }

            if (useBackMotorPid && backPidInitialized) {
                updateFieldCentricWithPID(forward, strafe, turn, heading);
            } else {
                updateFieldCentric(forward, strafe, turn, heading);
            }
        } else {
            // Robot centric
            if (useBackMotorPid && backPidInitialized) {
                updateRobotCentricWithPID(forward, strafe, turn);
            } else {
                updateRobotCentric(forward, strafe, turn);
            }
        }
    }

    /**
     * Robot centric with back motor PID
     */
    public void updateRobotCentricWithPID(double forward, double strafe, double turn) {
        boolean turning = Math.abs(turn) > 0.1;

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx frontLeftEx = (DcMotorEx) frontLeftDrive;
        DcMotorEx frontRightEx = (DcMotorEx) frontRightDrive;

        double frontLeftVelocity = frontLeftEx.getVelocity();
        double frontRightVelocity = frontRightEx.getVelocity();

        double frontLeftRPM = (frontLeftVelocity / DRIVE_TICKS_PER_REV) * 60.0;
        double frontRightRPM = (frontRightVelocity / DRIVE_TICKS_PER_REV) * 60.0;

        frontLeftDrive.setPower(forward + strafe + turn);
        frontRightDrive.setPower(-forward - strafe + turn);

        if (!turning) {
            backLeftController.setTargetRPM(Math.abs(frontRightRPM));
            backRightController.setTargetRPM(Math.abs(frontLeftRPM));
        } else {
            backLeftController.setTargetRPM(Math.abs(frontLeftRPM));
            backRightController.setTargetRPM(Math.abs(frontRightRPM));
        }

        backLeftController.loop();
        backRightController.loop();
    }

    /**
     * Robot centric
     */
    public void updateRobotCentric(double forward, double strafe, double turn) {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeftDrive.setPower(forward - strafe + turn);
        backRightDrive.setPower(-forward + strafe + turn);
        frontLeftDrive.setPower(forward + strafe + turn);
        frontRightDrive.setPower(-forward - strafe + turn);
    }

    /**
     * Field centric with back motor PID
     */
    public void updateFieldCentricWithPID(double forward, double strafe, double turn, double heading) {
        boolean turning = Math.abs(turn) > 0.1;

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Rotate movement vector
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        rotX = rotX * 1.1;

        // Get front motor velocities
        DcMotorEx frontLeftEx = (DcMotorEx) frontLeftDrive;
        DcMotorEx frontRightEx = (DcMotorEx) frontRightDrive;

        double frontLeftVelocity = frontLeftEx.getVelocity();
        double frontRightVelocity = frontRightEx.getVelocity();

        double frontLeftRPM = (frontLeftVelocity / DRIVE_TICKS_PER_REV) * 60.0;
        double frontRightRPM = (frontRightVelocity / DRIVE_TICKS_PER_REV) * 60.0;

        // Set front motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        frontLeftDrive.setPower((rotY + rotX + turn) / denominator);
        frontRightDrive.setPower((rotY - rotX - turn) / denominator);

        // Match back motors with PID
        if (!turning) {
            backLeftController.setTargetRPM(Math.abs(frontRightRPM));
            backRightController.setTargetRPM(Math.abs(frontLeftRPM));
        } else {
            backLeftController.setTargetRPM(Math.abs(frontLeftRPM));
            backRightController.setTargetRPM(Math.abs(frontRightRPM));
        }

        backLeftController.loop();
        backRightController.loop();
    }

    /**
     * Field centric
     */
    public void updateFieldCentric(double forward, double strafe, double turn, double heading) {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Apply wheel brake - uses public variables
     */
    public void applyWheelBrake() {
        int errorFL = wheelBrakeTargetFL - frontLeftDrive.getCurrentPosition();
        int errorFR = wheelBrakeTargetFR - frontRightDrive.getCurrentPosition();
        int errorBL = wheelBrakeTargetBL - backLeftDrive.getCurrentPosition();
        int errorBR = wheelBrakeTargetBR - backRightDrive.getCurrentPosition();

        errorFL = Math.max(-wheelBrakeMaxError, Math.min(wheelBrakeMaxError, errorFL));
        errorFR = Math.max(-wheelBrakeMaxError, Math.min(wheelBrakeMaxError, errorFR));
        errorBL = Math.max(-wheelBrakeMaxError, Math.min(wheelBrakeMaxError, errorBL));
        errorBR = Math.max(-wheelBrakeMaxError, Math.min(wheelBrakeMaxError, errorBR));

        double powerFL = Math.max(-wheelBrakeMaxPower, Math.min(wheelBrakeMaxPower, wheelBrakeKP * errorFL));
        double powerFR = Math.max(-wheelBrakeMaxPower, Math.min(wheelBrakeMaxPower, wheelBrakeKP * errorFR));
        double powerBL = Math.max(-wheelBrakeMaxPower, Math.min(wheelBrakeMaxPower, wheelBrakeKP * errorBL));
        double powerBR = Math.max(-wheelBrakeMaxPower, Math.min(wheelBrakeMaxPower, wheelBrakeKP * errorBR));

        frontLeftDrive.setPower(powerFL);
        frontRightDrive.setPower(powerFR);
        backLeftDrive.setPower(powerBL);
        backRightDrive.setPower(powerBR);
    }

    /**
     * Stop all motors
     */
    public void stopAllMotors() {
        if (frontLeftDrive != null) frontLeftDrive.setPower(0);
        if (frontRightDrive != null) frontRightDrive.setPower(0);
        if (backLeftDrive != null) backLeftDrive.setPower(0);
        if (backRightDrive != null) backRightDrive.setPower(0);
    }

    /**
     * Initialize wheel brake - call this from main teleop when activating brake
     */
    public void initWheelBrake() {
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setTargetPosition(0);
        frontRightDrive.setTargetPosition(0);
        backLeftDrive.setTargetPosition(0);
        backRightDrive.setTargetPosition(0);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheelBrakeTargetFL = frontLeftDrive.getCurrentPosition();
        wheelBrakeTargetFR = frontRightDrive.getCurrentPosition();
        wheelBrakeTargetBL = backLeftDrive.getCurrentPosition();
        wheelBrakeTargetBR = backRightDrive.getCurrentPosition();
    }

    // ==================== GETTERS ====================

    public boolean getInitialized() { return initialized; }
    public boolean getEnabled() { return enabled; }
    public double getBackLeftTargetRPM() { return backLeftController != null ? backLeftController.getTargetRPM() : 0; }
    public double getBackLeftCurrentRPM() { return backLeftController != null ? backLeftController.getCurrentRPM() : 0; }
    public double getBackRightTargetRPM() { return backRightController != null ? backRightController.getTargetRPM() : 0; }
    public double getBackRightCurrentRPM() { return backRightController != null ? backRightController.getCurrentRPM() : 0; }
}