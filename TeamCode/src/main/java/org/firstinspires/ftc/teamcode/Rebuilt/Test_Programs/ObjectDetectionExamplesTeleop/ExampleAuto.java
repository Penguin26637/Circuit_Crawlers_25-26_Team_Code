package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.ObjectDetectionExamplesTeleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Example Autonomous that uses the ObeliskIntakeSystem class
 * 
 * This demonstrates how to integrate the intake system into your autonomous code
 */
@Autonomous(name = "Auto with Intake System", group = "Examples")
public class ExampleAuto extends LinearOpMode {
    
    // Your existing robot hardware
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intakeMotor;
    
    // The intake decision system
    private ObeliskIntakeSystem intakeSystem;
    
    @Override
    public void runOpMode() {
        
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        
        // Initialize the intake system
        intakeSystem = new ObeliskIntakeSystem(hardwareMap);
        
        if (!intakeSystem.isInitialized()) {
            telemetry.addData("ERROR", "Intake system not initialized!");
            telemetry.update();
        }
        
        telemetry.addData("Status", "Initialized - waiting for start");
        intakeSystem.sendTelemetry(telemetry);
        telemetry.update();
        
        waitForStart();
        
        // ========== AUTONOMOUS SEQUENCE ==========
        
        // Step 1: Scan the obelisk at the start
        telemetry.addData("Status", "Scanning obelisk...");
        telemetry.update();
        
        // Force an immediate obelisk scan
        intakeSystem.forceObeliskUpdate();
        sleep(1000); // Give it time to detect
        
        // Step 2: Get the pattern and plan strategy
        String pattern = intakeSystem.getObeliskPattern();
        telemetry.addData("Detected Pattern", pattern);
        telemetry.update();
        
        // Step 3: Navigate to ball collection area
        driveForward(1.0, 2000); // Drive forward 2 seconds
        
        // Step 4: Collect balls with smart intake
        collectBalls(5000); // Collect for 5 seconds
        
        // Step 5: Navigate to scoring area
        turnRight(90); // Turn 90 degrees
        driveForward(1.0, 1000); // Drive to goal
        
        // Step 6: Score
        scoreInGoal();
        
        telemetry.addData("Status", "Autonomous Complete!");
        telemetry.update();
        
        // Cleanup
        intakeSystem.stop();
    }
    
    // ========== HELPER METHODS ==========
    
    /**
     * Collects balls for a specified duration using smart intake
     */
    private void collectBalls(long durationMs) {
        long startTime = System.currentTimeMillis();
        
        telemetry.addData("Status", "Collecting balls...");
        telemetry.update();
        
        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < durationMs) {
            
            // Update the intake system
            intakeSystem.update();
            
            // Smart intake control
            if (intakeSystem.shouldPickup()) {
                intakeMotor.setPower(1.0);  // Run intake for correct color
                telemetry.addData("Intake", "RUNNING - %s ball", 
                    intakeSystem.getDetectedBallColor());
            } else {
                if (!intakeSystem.getDetectedBallColor().equals("None")) {
                    // Ball detected but wrong color - reverse to reject
                    intakeMotor.setPower(-0.5);
                    telemetry.addData("Intake", "REJECTING - %s ball", 
                        intakeSystem.getDetectedBallColor());
                } else {
                    // No ball detected - idle
                    intakeMotor.setPower(0.0);
                    telemetry.addData("Intake", "IDLE - searching");
                }
            }
            
            // Show pattern
            telemetry.addData("Pattern", intakeSystem.getObeliskPattern());
            telemetry.update();
            
            sleep(20);
        }
        
        intakeMotor.setPower(0); // Stop intake
    }
    
    /**
     * Drives forward at specified power for specified time
     */
    private void driveForward(double power, long durationMs) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(durationMs);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    /**
     * Turns right by approximate degrees (simple time-based turn)
     */
    private void turnRight(int degrees) {
        // Simplified turn - in real code, use encoders or gyro
        long turnTime = (long)(degrees * 10); // 10ms per degree (tune this!)
        
        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        sleep(turnTime);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    /**
     * Scores collected balls in goal
     */
    private void scoreInGoal() {
        telemetry.addData("Status", "Scoring...");
        telemetry.update();
        
        // Run intake in reverse to eject balls
        intakeMotor.setPower(-1.0);
        sleep(2000);
        intakeMotor.setPower(0);
    }
}
