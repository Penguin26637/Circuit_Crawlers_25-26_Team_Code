
package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.RandomTestFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="INTAKETESTMODE", group="Linear OpMode")
public class INTAKETESTMODE extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftIntake = hardwareMap.get(CRServo.class, "leftintake");
        rightIntake = hardwareMap.get(CRServo.class, "rightintake");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            leftIntake.setPower(1);
            rightIntake.setPower(1);

            telemetry.addData("Left Intake", "Spinning at 1.0");
            telemetry.addData("Right Intake", "Spinning at 1.0");
            telemetry.update();
        }
    }
}