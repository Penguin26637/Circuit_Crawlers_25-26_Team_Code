package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * IntakeClass - Handles intake servo/motor logic
 * Supports 1 or 2 intakes, each can be Servo, CRServo, or DcMotor
 */
public class IntakeClass {

    // Hardware
    public Servo intake1Servo;
    public Servo intake2Servo;
    public CRServo intake1CRServo;
    public CRServo intake2CRServo;
    public DcMotor intake1Motor;
    public DcMotor intake2Motor;

    public Telemetry telemetry;

    // Control variables
    public double targetPosition = 0;  // For regular servos (0-1)
    public double targetPower = 0;     // For CR servos and motors (-1 to 1)

    // Component status
    public boolean intake1Initialized = false;
    public boolean intake2Initialized = false;
    public boolean initialized = false;

    // Type tracking
    public String intake1Type = "none";  // "servo", "crservo", "motor", or "none"
    public String intake2Type = "none";

    // Enabled flag
    public boolean enabled = false;

    public IntakeClass(HardwareMap hardwareMap, Telemetry telemetry, String initIntake1Type, String initIntake2Type) {
        this.telemetry = telemetry;

        // Initialize intake 1
        if (!initIntake1Type.equals("none")) {
            try {
                switch (initIntake1Type) {
                    case "servo":
                        intake1Servo = hardwareMap.get(Servo.class, "i");
                        intake1Servo.setPosition(0);
                        intake1Type = "servo";
                        intake1Initialized = true;
                        break;
                    case "crservo":
                        intake1CRServo = hardwareMap.get(CRServo.class, "i");
                        intake1CRServo.setPower(0);
                        intake1Type = "crservo";
                        intake1Initialized = true;
                        break;
                    case "motor":
                        intake1Motor = hardwareMap.get(DcMotor.class, "i");
                        intake1Motor.setPower(0);
                        intake1Type = "motor";
                        intake1Initialized = true;
                        break;
                }
            } catch (Exception e) {
                intake1Initialized = false;
                telemetry.addData("Intake 1 Error", e.getMessage());
            }
        }

        // Initialize intake 2
        if (!initIntake2Type.equals("none")) {
            try {
                switch (initIntake2Type) {
                    case "servo":
                        intake2Servo = hardwareMap.get(Servo.class, "i2");
                        intake2Servo.setPosition(0);
                        intake2Type = "servo";
                        intake2Initialized = true;
                        break;
                    case "crservo":
                        intake2CRServo = hardwareMap.get(CRServo.class, "i2");
                        intake2CRServo.setPower(0);
                        intake2Type = "crservo";
                        intake2Initialized = true;
                        break;
                    case "motor":
                        intake2Motor = hardwareMap.get(DcMotor.class, "i2");
                        intake2Motor.setPower(0);
                        intake2Type = "motor";
                        intake2Initialized = true;
                        break;
                }
            } catch (Exception e) {
                intake2Initialized = false;
                telemetry.addData("Intake 2 Error", e.getMessage());
            }
        }

        initialized = intake1Initialized || intake2Initialized;
    }

    public void update(boolean enabled) {
        this.enabled = enabled;

        if (!enabled || !initialized) {
            stopAll();
            return;
        }

        // Update intake 1
        if (intake1Initialized) {
            switch (intake1Type) {
                case "servo":
                    intake1Servo.setPosition(targetPosition);
                    break;
                case "crservo":
                    intake1CRServo.setPower(targetPower);
                    break;
                case "motor":
                    intake1Motor.setPower(targetPower);
                    break;
            }
        }

        // Update intake 2
        if (intake2Initialized) {
            switch (intake2Type) {
                case "servo":
                    intake2Servo.setPosition(targetPosition);
                    break;
                case "crservo":
                    intake2CRServo.setPower(targetPower);
                    break;
                case "motor":
                    intake2Motor.setPower(targetPower);
                    break;
            }
        }
    }

    public void stopAll() {
        if (intake1Initialized) {
            switch (intake1Type) {
                case "servo": intake1Servo.setPosition(0); break;
                case "crservo": intake1CRServo.setPower(0); break;
                case "motor": intake1Motor.setPower(0); break;
            }
        }
        if (intake2Initialized) {
            switch (intake2Type) {
                case "servo": intake2Servo.setPosition(0); break;
                case "crservo": intake2CRServo.setPower(0); break;
                case "motor": intake2Motor.setPower(0); break;
            }
        }
    }

    // ==================== GETTERS ====================

    public boolean getInitialized() { return initialized; }
    public boolean getEnabled() { return enabled; }
    public boolean getIntake1Initialized() { return intake1Initialized; }
    public boolean getIntake2Initialized() { return intake2Initialized; }

    /**
     * Returns the current value of the specified intake.
     * For Servo: returns position (0-1)
     * For CRServo or DcMotor: returns power (-1 to 1)
     * @param intakeNumber 1 or 2
     * @return current position or power, 0 if not initialized
     */
    public double getIntakeValue(int intakeNumber) {
        if (intakeNumber == 1 && intake1Initialized) {
            switch (intake1Type) {
                case "servo": return intake1Servo != null ? intake1Servo.getPosition() : 0;
                case "crservo": return intake1CRServo != null ? intake1CRServo.getPower() : 0;
                case "motor": return intake1Motor != null ? intake1Motor.getPower() : 0;
            }
        } else if (intakeNumber == 2 && intake2Initialized) {
            switch (intake2Type) {
                case "servo": return intake2Servo != null ? intake2Servo.getPosition() : 0;
                case "crservo": return intake2CRServo != null ? intake2CRServo.getPower() : 0;
                case "motor": return intake2Motor != null ? intake2Motor.getPower() : 0;
            }
        }
        return 0;
    }

    public double getIntake1Value() { return getIntakeValue(1); }
    public double getIntake2Value() { return getIntakeValue(2); }
}
