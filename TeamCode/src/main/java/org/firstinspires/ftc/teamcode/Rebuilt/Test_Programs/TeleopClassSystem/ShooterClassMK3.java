package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.TeleopClassSystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Rebuilt.MainPrograms.MotorPowerRegulator_New;

/**
 * ShooterClassMK3 - Handles shooter logic
 * Supports flexible magazine: up to 4, each can be Servo, CRServo, or DcMotor
 * Added spindexer and flipper servos
 * All variables are public and controlled from main teleop
 */
public class ShooterClassMK3 {

    // Hardware - Public - Only initialized ones will be non-null
    public MotorPowerRegulator_New shooterMotor;

    public Servo hinge1Servo, hinge2Servo;
    public CRServo hinge1CRservo, hinge2CRservo;
    public DcMotor hinge1Motor, hinge2Motor;
    public CRServo magazine1CRServo, magazine2CRServo, magazine3CRServo, magazine4CRServo;
    public Servo magazine1Servo, magazine2Servo, magazine3Servo, magazine4Servo;
    public DcMotor magazine1Motor, magazine2Motor, magazine3Motor, magazine4Motor;

    // New servos for MK3
    public CRServo spindexerCRServo;
    public Servo spindexerServo;
    public DcMotor spindexerMotor;
    public CRServo flipperCRServo;
    public Servo flipperServo;
    public DcMotor flipperMotor;

    public Telemetry telemetry;

    // Control variables - Public - Set these from main teleop
    public double shooterTargetRPM = 0;
    public double hingeTargetPosition = 0;
    public double hingeTargetPower = 0;
    public double magazineTargetPower = 0;      // For CR servos and motors (-1 to 1)
    public double magazineTargetPosition = 0;   // For regular servos (0-1)

    // New control variables for spindexer and flipper
    public double spindexerTargetPower = 0;      // For CR servos and motors (-1 to 1)
    public double spindexerTargetPosition = 0;   // For regular servos (0-1)
    public double flipperTargetPower = 0;        // For CR servos and motors (-1 to 1)
    public double flipperTargetPosition = 0;     // For regular servos (0-1)

    // Component status - Public
    public boolean shooterEnabled = true;
    public boolean hingeEnabled = false;
    public boolean magazineEnabled = false;
    public boolean spindexerEnabled = false;
    public boolean flipperEnabled = false;
    public boolean shooterInitialized = false;
    public boolean hinge1Initialized = false;
    public boolean hinge2Initialized = false;
    public boolean magazine1Initialized = false;
    public boolean magazine2Initialized = false;
    public boolean magazine3Initialized = false;
    public boolean magazine4Initialized = false;
    public boolean spindexerInitialized = false;
    public boolean flipperInitialized = false;
    public boolean magazineInitialized = false;
    public boolean hingeInitialized = false;

    // Magazine type tracking - Public - Set from teleop constructor parameters
    public String magazine1Type;
    public String magazine2Type;
    public String magazine3Type;
    public String magazine4Type;
    public String hinge1Type;
    public String hinge2Type;
    public String spindexerType;
    public String flipperType;

    /**
     * Constructor with flexible magazine types and new servos
     * @param initShooter Initialize shooter motor
     * @param initHinge1Type Initialize hinge servo
     * @param initHinge2Type Initialize hinge servo
     * @param initMag1Type "servo", "crservo", "motor", or "none"
     * @param initMag2Type "servo", "crservo", "motor", or "none"
     * @param initMag3Type "servo", "crservo", "motor", or "none"
     * @param initMag4Type "servo", "crservo", "motor", or "none"
     * @param initSpindexerType "servo", "crservo", "motor", or "none"
     * @param initFlipperType "servo", "crservo", "motor", or "none"
     */
    public ShooterClassMK3(HardwareMap hardwareMap, Telemetry telemetry,
                           boolean initShooter, String initHinge1Type, String initHinge2Type,
                           String initMag1Type, String initMag2Type, String initMag3Type, String initMag4Type,
                           String initSpindexerType, String initFlipperType) {
        this.telemetry = telemetry;

        // Initialize shooter motor
        if (initShooter) {
            try {
                shooterMotor = new MotorPowerRegulator_New(hardwareMap, telemetry, "s");
                shooterMotor.setTicksPerRev(112.0);
                shooterMotor.setMaxRpmUnderLoad(600);
                shooterMotor.setAllGains(0.0006785714285714286, 0.06, 0.0004, 0.0002, 0.00005);
                shooterInitialized = true;
            } catch (Exception e) {
                shooterInitialized = false;
                telemetry.addData("Shooter Error", e.getMessage());
            }
        }

        // Initialize hinge servo
        if (!initHinge1Type.equals("none")) {
            try {
                if (initHinge1Type.equals("servo")) {
                    hinge1Servo = hardwareMap.get(Servo.class, "hinge1");
                    hinge1Servo.setPosition(0);
                    hinge1Type = "servo";
                    hinge1Initialized = true;
                } else if (initHinge1Type.equals("crservo")) {
                    hinge1CRservo = hardwareMap.get(CRServo.class, "hinge1");
                    hinge1CRservo.setPower(0);
                    hinge1Type = "crservo";
                    hinge1Initialized = true;
                } else if (initHinge1Type.equals("motor")) {
                    hinge1Motor = hardwareMap.get(DcMotor.class, "hinge1");
                    hinge1Motor.setPower(0);
                    hinge1Type = "motor";
                    hinge1Initialized = true;
                }
            } catch (Exception e) {
                hinge1Initialized = false;
                telemetry.addData("hinge 1 Error", e.getMessage());
            }
        }
        if (!initHinge2Type.equals("none")) {
            try {
                if (initHinge2Type.equals("servo")) {
                    hinge2Servo = hardwareMap.get(Servo.class, "hinge2");
                    hinge2Servo.setPosition(0);
                    hinge2Type = "servo";
                    hinge2Initialized = true;
                } else if (initHinge2Type.equals("crservo")) {
                    hinge2CRservo = hardwareMap.get(CRServo.class, "hinge2");
                    hinge2CRservo.setPower(0);
                    hinge2Type = "crservo";
                    hinge2Initialized = true;
                } else if (initHinge2Type.equals("motor")) {
                    hinge2Motor = hardwareMap.get(DcMotor.class, "hinge2");
                    hinge2Motor.setPower(0);
                    hinge2Type = "motor";
                    hinge2Initialized = true;
                }
            } catch (Exception e) {
                hinge2Initialized = false;
                telemetry.addData("hinge 2 Error", e.getMessage());
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
                telemetry.addData("Magazine 3 Error", e.getMessage());
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

        // Initialize spindexer (hardware name: "spin")
        if (!initSpindexerType.equals("none")) {
            try {
                if (initSpindexerType.equals("servo")) {
                    spindexerServo = hardwareMap.get(Servo.class, "spin");
                    spindexerServo.setPosition(0);
                    spindexerType = "servo";
                    spindexerInitialized = true;
                } else if (initSpindexerType.equals("crservo")) {
                    spindexerCRServo = hardwareMap.get(CRServo.class, "spin");
                    spindexerCRServo.setPower(0);
                    spindexerType = "crservo";
                    spindexerInitialized = true;
                } else if (initSpindexerType.equals("motor")) {
                    spindexerMotor = hardwareMap.get(DcMotor.class, "spin");
                    spindexerMotor.setPower(0);
                    spindexerType = "motor";
                    spindexerInitialized = true;
                }
            } catch (Exception e) {
                spindexerInitialized = false;
                telemetry.addData("Spindexer Error", e.getMessage());
            }
        }

        // Initialize flipper (hardware name: "flipper")
        if (!initFlipperType.equals("none")) {
            try {
                if (initFlipperType.equals("servo")) {
                    flipperServo = hardwareMap.get(Servo.class, "flipper");
                    flipperServo.setPosition(0);
                    flipperType = "servo";
                    flipperInitialized = true;
                } else if (initFlipperType.equals("crservo")) {
                    flipperCRServo = hardwareMap.get(CRServo.class, "flipper");
                    flipperCRServo.setPower(0);
                    flipperType = "crservo";
                    flipperInitialized = true;
                } else if (initFlipperType.equals("motor")) {
                    flipperMotor = hardwareMap.get(DcMotor.class, "flipper");
                    flipperMotor.setPower(0);
                    flipperType = "motor";
                    flipperInitialized = true;
                }
            } catch (Exception e) {
                flipperInitialized = false;
                telemetry.addData("Flipper Error", e.getMessage());
            }
        }

        magazineInitialized = magazine1Initialized || magazine2Initialized || magazine3Initialized || magazine4Initialized;
        hingeInitialized = hinge1Initialized || hinge2Initialized;
    }

    /**
     * Main update - reads public variables and applies them
     */
    public void update(boolean shooterEnabled, boolean hingeEnabled, boolean magazineEnabled,
                       boolean spindexerEnabled, boolean flipperEnabled) {
        this.shooterEnabled = shooterEnabled;
        this.hingeEnabled = hingeEnabled;
        this.magazineEnabled = magazineEnabled;
        this.spindexerEnabled = spindexerEnabled;
        this.flipperEnabled = flipperEnabled;

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
        if (hingeEnabled && hinge1Initialized) {
            if (hinge1Type.equals("servo")) {
                hinge1Servo.setPosition(hingeTargetPosition);
            } else if (hinge1Type.equals("crservo")) {
                hinge1CRservo.setPower(hingeTargetPower);
            } else if (hinge1Type.equals("motor")) {
                hinge1Motor.setPower(hingeTargetPower);
            }
        } else if (hinge1Initialized) {
            stophinge1();
        }
        if (hingeEnabled && hinge2Initialized) {
            if (hinge2Type.equals("servo")) {
                hinge2Servo.setPosition(hingeTargetPosition);
            } else if (hinge2Type.equals("crservo")) {
                hinge2CRservo.setPower(hingeTargetPower);
            } else if (hinge2Type.equals("motor")) {
                hinge2Motor.setPower(hingeTargetPower);
            }
        } else if (hinge2Initialized) {
            stophinge2();
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

        // Update spindexer
        if (spindexerEnabled && spindexerInitialized) {
            if (spindexerType.equals("servo")) {
                spindexerServo.setPosition(spindexerTargetPosition);
            } else if (spindexerType.equals("crservo")) {
                spindexerCRServo.setPower(spindexerTargetPower);
            } else if (spindexerType.equals("motor")) {
                spindexerMotor.setPower(spindexerTargetPower);
            }
        } else if (spindexerInitialized) {
            stopSpindexer();
        }

        // Update flipper
        if (flipperEnabled && flipperInitialized) {
            if (flipperType.equals("servo")) {
                flipperServo.setPosition(flipperTargetPosition);
            } else if (flipperType.equals("crservo")) {
                flipperCRServo.setPower(flipperTargetPower);
            } else if (flipperType.equals("motor")) {
                flipperMotor.setPower(flipperTargetPower);
            }
        } else if (flipperInitialized) {
            stopFlipper();
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
    public void stophinge1() {
        if (hinge1Type.equals("servo")) {
            hinge1Servo.setPosition(0);
        } else if (hinge1Type.equals("crservo")) {
            hinge1CRservo.setPower(0);
        } else if (hinge1Type.equals("motor")) {
            hinge1Motor.setPower(0);
        }
    }
    public void stophinge2() {
        if (hinge2Type.equals("servo")) {
            hinge2Servo.setPosition(0);
        } else if (hinge2Type.equals("crservo")) {
            hinge2CRservo.setPower(0);
        } else if (hinge2Type.equals("motor")) {
            hinge2Motor.setPower(0);
        }
    }

    /**
     * Stop spindexer
     */
    public void stopSpindexer() {
        if (spindexerType.equals("servo")) {
            spindexerServo.setPosition(0);
        } else if (spindexerType.equals("crservo")) {
            spindexerCRServo.setPower(0);
        } else if (spindexerType.equals("motor")) {
            spindexerMotor.setPower(0);
        }
    }

    /**
     * Stop flipper
     */
    public void stopFlipper() {
        if (flipperType.equals("servo")) {
            flipperServo.setPosition(0);
        } else if (flipperType.equals("crservo")) {
            flipperCRServo.setPower(0);
        } else if (flipperType.equals("motor")) {
            flipperMotor.setPower(0);
        }
    }

    // ==================== GETTERS ====================

    public boolean getShooterInitialized() { return shooterInitialized; }
    public boolean getMagazineInitialized() { return magazineInitialized; }
    public boolean getMagazine1Initialized() { return magazine1Initialized; }
    public boolean getMagazine2Initialized() { return magazine2Initialized; }
    public boolean getMagazine3Initialized() { return magazine3Initialized; }
    public boolean getMagazine4Initialized() { return magazine4Initialized; }
    public boolean getSpindexerInitialized() { return spindexerInitialized; }
    public boolean getFlipperInitialized() { return flipperInitialized; }
    public boolean getShooterEnabled() { return shooterEnabled; }
    public boolean getHinge1Initialized() { return hinge1Initialized; }
    public boolean getHinge2Initialized() { return hinge2Initialized; }

    public boolean getMagazineEnabled() { return magazineEnabled; }
    public boolean getSpindexerEnabled() { return spindexerEnabled; }
    public boolean getFlipperEnabled() { return flipperEnabled; }

    public double getShooterCurrentRPM() {
        return shooterMotor != null ? shooterMotor.getCurrentRPM() : 0;
    }

    public boolean getShooterAtTarget(double tolerance) {
        return shooterMotor != null && shooterMotor.isAtTarget(tolerance);
    }
}