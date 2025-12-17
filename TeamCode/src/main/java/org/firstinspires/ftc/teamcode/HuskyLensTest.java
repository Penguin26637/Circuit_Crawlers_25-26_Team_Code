package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp(name = "HuskyLensTest", group = "Camera Vision")
public class HuskyLensTest extends LinearOpMode {

    private static final int READ_PERIOD = 1; // seconds
    private static final String DIVIDER =
            "----------------------------------------------------------------";

    private HuskyLens huskyLens;
    private FtcDashboard dashboard;

    // Seen flags
    private boolean block1Seen;
    private boolean block2Seen;
    private boolean block3Seen;

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        huskyLens = hardwareMap.get(HuskyLens.class, "hl");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) continue;
            rateLimit.reset();

            // Reset flags
            block1Seen = false;
            block2Seen = false;
            block3Seen = false;

            telemetry.clear();
            TelemetryPacket packet = new TelemetryPacket();

            HuskyLens.Block[] blocks = huskyLens.blocks();

            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Target Visible", blocks.length > 0);
            telemetry.addLine(DIVIDER);

            packet.put("Block count", blocks.length);
            packet.put("Target Visible", blocks.length > 0);

            // ---- PROCESS DETECTED BLOCKS ----
            for (HuskyLens.Block block : blocks) {
                processBlock(block, packet, block1Seen, block2Seen, block3Seen);
            }


//            if (block1Seen){
////                auto code here
//
//            } else if (block2Seen) {
////                auto code here
//            } else if (block3Seen) {
////                auto code here
//            }

            // ---- CLEAR / FILL MISSING BLOCKS ----
            updateDashboardAndTelemetry(packet, block1Seen, block2Seen, block3Seen);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    private void processBlock(HuskyLens.Block block, TelemetryPacket packet, boolean b1, boolean b2, boolean b3) {



        if (block.id < 1 || block.id > 3) return;

        if (block.id == 1) b1 = true;
        if (block.id == 2) b2 = true;
        if (block.id == 3) b3 = true;
        if (b1) {
            telemetry.addLine("Block " + block.id);
            telemetry.addData("X Center", block.x);
            telemetry.addData("Y Center", block.y);
            telemetry.addData("Width", block.width);
            telemetry.addData("Height", block.height);
            telemetry.addLine(DIVIDER);

            packet.put("B" + "1" + "/Status", "DETECTED");
            packet.put("B" + "1" + "/X Center", block.x);
            packet.put("B" + "1" + "/Y Center", block.y);
            packet.put("B" + "1" + "/Width", block.width);
            packet.put("B" + "1" + "/Height", block.height);
            packet.addLine("\n");
        }
        else {
            packet.put(" ", " ");
        }
        if (b2) {
            telemetry.addLine("Block " + block.id);
            telemetry.addData("X Center", block.x);
            telemetry.addData("Y Center", block.y);
            telemetry.addData("Width", block.width);
            telemetry.addData("Height", block.height);
            telemetry.addLine(DIVIDER);

            packet.put("B" + "2" + "/Status", "DETECTED");
            packet.put("B" + "2" + "/X Center", block.x);
            packet.put("B" + "2" + "/Y Center", block.y);
            packet.put("B" + "2" + "/Width", block.width);
            packet.put("B" + "2" + "/Height", block.height);
            packet.addLine("\n");
        }
        else {
            packet.put(" ", " ");
        }
        if (b3) {
            telemetry.addLine("Block " + block.id);
            telemetry.addData("X Center", block.x);
            telemetry.addData("Y Center", block.y);
            telemetry.addData("Width", block.width);
            telemetry.addData("Height", block.height);
            telemetry.addLine(DIVIDER);

            packet.put("B" + "3" + "/Status", "DETECTED");
            packet.put("B" + "3" + "/X Center", block.x);
            packet.put("B" + "3" + "/Y Center", block.y);
            packet.put("B" + "3" + "/Width", block.width);
            packet.put("B" + "3" + "/Height", block.height);
            packet.addLine("\n\n");
        }
        else {
            packet.put(" ", " ");
        }

    }

    private void updateDashboardAndTelemetry(TelemetryPacket packet, boolean b1, boolean b2, boolean b3) {
        updateBlock(1, b1, packet);
        updateBlock(2, b2, packet);
        updateBlock(3, b3, packet);
    }

    private void updateBlock(int id, boolean seen, TelemetryPacket packet) {

        if (!seen) {
            telemetry.addLine("Block " + id + ": —");
            telemetry.addLine(DIVIDER);

            packet.put("B" + id + "/Status", "—");


        }
    }
}
