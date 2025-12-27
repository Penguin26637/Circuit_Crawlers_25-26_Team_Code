package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

/**
 * ShooterClass - Handles shooter logic
 * Supports flexible magazine: 1 or 2, each can be Servo, CRServo, or DcMotor
 * All variables are public and controlled from main teleop
 */
public class ShooterClass {

    // Hardware - Public - Only initialized ones will be non-null
    public MotorPowerRegulator_New shooterMotor;

    public Servo hinge1, hinge2;
    public CRServo magazine1CRServo, magazine2CRServo, magazine3CRServo, magazine4CRServo;
    public Servo magazine1Servo, magazine2Servo, magazine3Servo, magazine4Servo;
    public DcMotor magazine1Motor, magazine2Motor, magazine3Motor, magazine4Motor;

    public Telemetry telemetry;

    // Control variables - Public - Set these from main teleop
    public double shooterTargetRPM = 0;
    public double hinge1TargetPosition = 0;
    public double hinge2TargetPosition = 0;
    public double magazineTargetPower = 0;      // For CR servos and motors (-1 to 1)
    public double magazineTargetPosition = 0;   // For regular servos (0-1)

    // Component status - Public
    public boolean shooterEnabled = false;
    public boolean hinge1Enabled = false;
    public boolean hinge2Enabled = false;
    public boolean magazineEnabled = false;
    public boolean shooterInitialized = false;
    public boolean hinge1Initialized = false;
    public boolean hinge2Initialized = false;
    public boolean magazine1Initialized = false;
    public boolean magazine2Initialized = false;
    public boolean magazine3Initialized = false;
    public boolean magazine4Initialized = false;
    public boolean magazineInitialized = false;

    // Magazine type tracking - Public
    public String magazine1Type = "none";  // "servo", "crservo", "motor", or "none"
    public String magazine2Type = "none";
    public String magazine3Type = "none";
    public String magazine4Type = "none";

    /**
     * Constructor with flexible magazine types
     * @param initShooter Initialize shooter motor
     * @param initHinge1 Initialize hinge serv
     * @param initHinge2 Initialize hinge servo
     * @param initMag1Type "servo", "crservo", "motor", or "none"
     * @param initMag2Type "servo", "crservo", "motor", or "none"
     */
    public ShooterClass(HardwareMap hardwareMap, Telemetry telemetry,
                        boolean initShooter, boolean initHinge1, boolean initHinge2,
                        String initMag1Type, String initMag2Type, String initMag3Type, String initMag4Type) {
        this.telemetry = telemetry;

        // Initialize shooter motor
        if (initShooter) {
            try {
                shooterMotor = new MotorPowerRegulator_New(hardwareMap, telemetry, "s");
                shooterMotor.setTicksPerRev(112.0);
                shooterMotor.setMaxRpmUnderLoad(1400.0);
                shooterMotor.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);
                shooterInitialized = true;
            } catch (Exception e) {
                shooterInitialized = false;
                telemetry.addData("Shooter Error", e.getMessage());
            }
        }

        // Initialize hinge servo
        if (initHinge1) {
            try {
                hinge1 = hardwareMap.get(Servo.class, "sH");
                hinge1.setPosition(0);
                hinge1Initialized = true;
            } catch (Exception e) {
                hinge1Initialized = false;
                telemetry.addData("hinge1 Error", e.getMessage());
            }
        }


        // Initialize magazine 1
        if (!initMag1Type.equals("none")) {
            try {
                if (initMag1Type.equals("servo")) {
                    magazine1Servo = hardwareMap.get(Servo.class, "mag1");
                    magazine1Servo.setPosition(0);
                    magazine1Type = "servo";
                    magazine1Initialized = true;
                } else if (initMag1Type.equals("crservo")) {
                    magazine1CRServo = hardwareMap.get(CRServo.class, "mag1");
                    magazine1CRServo.setPower(0);
                    magazine1Type = "crservo";
                    magazine1Initialized = true;
                } else if (initMag1Type.equals("motor")) {
                    magazine1Motor = hardwareMap.get(DcMotor.class, "mag1");
                    magazine1Motor.setPower(0);
                    magazine1Type = "motor";
                    magazine1Initialized = true;
                }
            } catch (Exception e) {
                magazine1Initialized = false;
                telemetry.addData("Magazine 1 Error", e.getMessage());
            }
        }

        // Initialize magazine 2
        if (!initMag2Type.equals("none")) {
            try {
                if (initMag2Type.equals("servo")) {
                    magazine2Servo = hardwareMap.get(Servo.class, "mag2");
                    magazine2Servo.setPosition(0);
                    magazine2Type = "servo";
                    magazine2Initialized = true;
                } else if (initMag2Type.equals("crservo")) {
                    magazine2CRServo = hardwareMap.get(CRServo.class, "mag2");
                    magazine2CRServo.setPower(0);
                    magazine2Type = "crservo";
                    magazine2Initialized = true;
                } else if (initMag2Type.equals("motor")) {
                    magazine2Motor = hardwareMap.get(DcMotor.class, "mag2");
                    magazine2Motor.setPower(0);
                    magazine2Type = "motor";
                    magazine2Initialized = true;
                }
            } catch (Exception e) {
                magazine2Initialized = false;
                telemetry.addData("Magazine 2 Error", e.getMessage());
            }
        }
        if (!initMag3Type.equals("none")) {
            try {
                if (initMag3Type.equals("servo")) {
                    magazine3Servo = hardwareMap.get(Servo.class, "mag3");
                    magazine3Servo.setPosition(0);
                    magazine3Type = "servo";
                    magazine3Initialized = true;
                } else if (initMag3Type.equals("crservo")) {
                    magazine3CRServo = hardwareMap.get(CRServo.class, "mag3");
                    magazine3CRServo.setPower(0);
                    magazine3Type = "crservo";
                    magazine3Initialized = true;
                } else if (initMag3Type.equals("motor")) {
                    magazine3Motor = hardwareMap.get(DcMotor.class, "mag3");
                    magazine3Motor.setPower(0);
                    magazine3Type = "motor";
                    magazine3Initialized = true;
                }
            } catch (Exception e) {
                magazine3Initialized = false;
                telemetry.addData("Magazine 4 Error", e.getMessage());
            }
        }
        if (!initMag4Type.equals("none")) {
            try {
                if (initMag4Type.equals("servo")) {
                    magazine4Servo = hardwareMap.get(Servo.class, "mag4");
                    magazine4Servo.setPosition(0);
                    magazine4Type = "servo";
                    magazine4Initialized = true;
                } else if (initMag4Type.equals("crservo")) {
                    magazine4CRServo = hardwareMap.get(CRServo.class, "mag4");
                    magazine4CRServo.setPower(0);
                    magazine4Type = "crservo";
                    magazine4Initialized = true;
                } else if (initMag4Type.equals("motor")) {
                    magazine4Motor = hardwareMap.get(DcMotor.class, "mag4");
                    magazine4Motor.setPower(0);
                    magazine4Type = "motor";
                    magazine4Initialized = true;
                }
            } catch (Exception e) {
                magazine4Initialized = false;
                telemetry.addData("Magazine 4 Error", e.getMessage());
            }
        }

        magazineInitialized = magazine1Initialized || magazine2Initialized || magazine3Initialized || magazine4Initialized;
    }

    /**
     * Main update - reads public variables and applies them
     */
    public void update(boolean shooterEnabled, boolean hinge1Enabled, boolean hinge2Enabled, boolean magazineEnabled) {
        this.shooterEnabled = shooterEnabled;
        this.hinge1Enabled = hinge1Enabled;
        this.hinge2Enabled = hinge2Enabled;
        this.magazineEnabled = magazineEnabled;

        // Update shooter motor
        if (shooterEnabled && shooterInitialized) {
            shooterMotor.setTargetRPM(shooterTargetRPM);
            if (shooterTargetRPM > 0) {
                shooterMotor.loop();
            } else {
                shooterMotor.stop();
            }
        } else if (shooterInitialized) {
            shooterMotor.stop();
        }

        // Update hinge
        if (hinge1Enabled && hinge1Initialized) {
            hinge1.setPosition(hinge1TargetPosition);
        } else if (hinge1Initialized) {
            hinge1.setPosition(0);
        }
        if (hinge2Enabled && hinge2Initialized) {
            hinge2.setPosition(hinge2TargetPosition);
        } else if (hinge2Initialized) {
            hinge2.setPosition(0);
        }

        // Update magazine 1
        if (magazineEnabled && magazine1Initialized) {
            if (magazine1Type.equals("servo")) {
                magazine1Servo.setPosition(magazineTargetPosition);
            } else if (magazine1Type.equals("crservo")) {
                magazine1CRServo.setPower(magazineTargetPower);
            } else if (magazine1Type.equals("motor")) {
                magazine1Motor.setPower(magazineTargetPower);
            }
        } else if (magazine1Initialized) {
            stopMagazine1();
        }

        // Update magazine 2
        if (magazineEnabled && magazine2Initialized) {
            if (magazine2Type.equals("servo")) {
                magazine2Servo.setPosition(magazineTargetPosition);
            } else if (magazine2Type.equals("crservo")) {
                magazine2CRServo.setPower(magazineTargetPower);
            } else if (magazine2Type.equals("motor")) {
                magazine2Motor.setPower(magazineTargetPower);
            }
        } else if (magazine2Initialized) {
            stopMagazine2();
        }
        if (magazineEnabled && magazine3Initialized) {
            if (magazine3Type.equals("servo")) {
                magazine3Servo.setPosition(magazineTargetPosition);
            } else if (magazine3Type.equals("crservo")) {
                magazine3CRServo.setPower(magazineTargetPower);
            } else if (magazine3Type.equals("motor")) {
                magazine3Motor.setPower(magazineTargetPower);
            }
        } else if (magazine3Initialized) {
            stopMagazine3();
        }
        if (magazineEnabled && magazine4Initialized) {
            if (magazine4Type.equals("servo")) {
                magazine4Servo.setPosition(magazineTargetPosition);
            } else if (magazine4Type.equals("crservo")) {
                magazine4CRServo.setPower(magazineTargetPower);
            } else if (magazine4Type.equals("motor")) {
                magazine4Motor.setPower(magazineTargetPower);
            }
        } else if (magazine4Initialized) {
            stopMagazine4();
        }
    }

    /**
     * Stop magazine 1
     */
    public void stopMagazine1() {
        if (magazine1Type.equals("servo")) {
            magazine1Servo.setPosition(0);
        } else if (magazine1Type.equals("crservo")) {
            magazine1CRServo.setPower(0);
        } else if (magazine1Type.equals("motor")) {
            magazine1Motor.setPower(0);
        }
    }

    /**
     * Stop magazine 2
     */
    public void stopMagazine2() {
        if (magazine2Type.equals("servo")) {
            magazine2Servo.setPosition(0);
        } else if (magazine2Type.equals("crservo")) {
            magazine2CRServo.setPower(0);
        } else if (magazine2Type.equals("motor")) {
            magazine2Motor.setPower(0);
        }
    }
    public void stopMagazine3() {
        if (magazine3Type.equals("servo")) {
            magazine3Servo.setPosition(0);
        } else if (magazine3Type.equals("crservo")) {
            magazine3CRServo.setPower(0);
        } else if (magazine3Type.equals("motor")) {
            magazine3Motor.setPower(0);
        }
    }
    public void stopMagazine4() {
        if (magazine4Type.equals("servo")) {
            magazine4Servo.setPosition(0);
        } else if (magazine4Type.equals("crservo")) {
            magazine4CRServo.setPower(0);
        } else if (magazine4Type.equals("motor")) {
            magazine4Motor.setPower(0);
        }
    }

    // ==================== GETTERS ====================

    public boolean getShooterInitialized() { return shooterInitialized; }
    public boolean getHinge1Initialized() { return hinge1Initialized; }
    public boolean getHinge2Initialized() { return hinge2Initialized; }
    public boolean getMagazineInitialized() { return magazineInitialized; }
    public boolean getMagazine1Initialized() { return magazine1Initialized; }
    public boolean getMagazine2Initialized() { return magazine2Initialized; }
    public boolean getMagazine3Initialized() { return magazine3Initialized; }
    public boolean getMagazine4Initialized() { return magazine4Initialized; }
    public boolean getShooterEnabled() { return shooterEnabled; }
    public boolean getHinge1Enabled() { return hinge1Enabled; }
    public boolean getHinge2Enabled() { return hinge2Enabled; }

    public boolean getMagazineEnabled() { return magazineEnabled; }

    public double getShooterCurrentRPM() {
        return shooterMotor != null ? shooterMotor.getCurrentRPM() : 0;
    }

    public boolean getShooterAtTarget(double tolerance) {
        return shooterMotor != null && shooterMotor.isAtTarget(tolerance);
    }

    public double getHinge1Position() {
        if (hinge1 != null) {
            return hinge1.getPosition();
        } else return 0;
    }


    public double getHinge2Position() {
        if (hinge2 != null) {
            return hinge2.getPosition();
        } else return 0;
    }
}


