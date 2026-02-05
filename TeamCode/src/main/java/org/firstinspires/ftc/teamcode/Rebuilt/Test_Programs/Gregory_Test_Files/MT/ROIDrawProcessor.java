package org.firstinspires.ftc.teamcode.Rebuilt.Test_Programs.Gregory_Test_Files.MT;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * ROIDrawProcessor - Draws the Region of Interest rectangle on the camera feed
 *
 * This processor overlays a green rectangle showing the active detection zone
 * and darkens the excluded areas to make the ROI clearly visible on FTC Dashboard.
 */
public class ROIDrawProcessor implements VisionProcessor {

    // ROI boundaries (0.0 to 1.0)
    private volatile double roiLeftPercent = 0.0;
    private volatile double roiTopPercent = 0.0;
    private volatile double roiRightPercent = 1.0;
    private volatile double roiBottomPercent = 1.0;

    private int frameWidth = 640;
    private int frameHeight = 360;

    // Colors
    private final Scalar ROI_COLOR = new Scalar(0, 255, 0);  // Green
    private final Scalar OVERLAY_COLOR = new Scalar(0, 0, 0, 128);  // Semi-transparent black
    private final int ROI_THICKNESS = 3;

    /**
     * Update the ROI boundaries
     * Call this method whenever ROI settings change
     */
    public void updateROI(double left, double top, double right, double bottom) {
        this.roiLeftPercent = left;
        this.roiTopPercent = top;
        this.roiRightPercent = right;
        this.roiBottomPercent = bottom;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        frameWidth = width;
        frameHeight = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Calculate pixel coordinates
        int roiLeft = (int)(roiLeftPercent * frameWidth);
        int roiTop = (int)(roiTopPercent * frameHeight);
        int roiRight = (int)(roiRightPercent * frameWidth);
        int roiBottom = (int)(roiBottomPercent * frameHeight);

        // Ensure valid bounds
        roiLeft = Math.max(0, Math.min(roiLeft, frameWidth - 1));
        roiRight = Math.max(roiLeft + 1, Math.min(roiRight, frameWidth));
        roiTop = Math.max(0, Math.min(roiTop, frameHeight - 1));
        roiBottom = Math.max(roiTop + 1, Math.min(roiBottom, frameHeight));

        // Draw semi-transparent overlay on excluded regions
        drawExcludedOverlay(frame, roiLeft, roiTop, roiRight, roiBottom);

        // Draw ROI rectangle border
        Imgproc.rectangle(frame,
                new Point(roiLeft, roiTop),
                new Point(roiRight, roiBottom),
                ROI_COLOR,
                ROI_THICKNESS);

        // Draw corner markers for better visibility
        drawCornerMarkers(frame, roiLeft, roiTop, roiRight, roiBottom);

        // Draw center crosshair
        int centerX = (roiLeft + roiRight) / 2;
        int centerY = (roiTop + roiBottom) / 2;
        drawCrosshair(frame, centerX, centerY);

        // Draw text labels
        String roiText = String.format("ROI: %dx%d", roiRight - roiLeft, roiBottom - roiTop);
        Imgproc.putText(frame, roiText,
                new Point(roiLeft + 10, roiTop + 25),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                0.6,
                ROI_COLOR,
                2);

        return null;
    }

    /**
     * Draw semi-transparent overlay on excluded regions
     */
    private void drawExcludedOverlay(Mat frame, int roiLeft, int roiTop, int roiRight, int roiBottom) {
        // Create overlay mat
        Mat overlay = frame.clone();

        // Darken the excluded regions
        Scalar darkColor = new Scalar(0, 0, 0);

        // Top region
        if (roiTop > 0) {
            Imgproc.rectangle(overlay,
                    new Point(0, 0),
                    new Point(frameWidth, roiTop),
                    darkColor,
                    -1);
        }

        // Bottom region
        if (roiBottom < frameHeight) {
            Imgproc.rectangle(overlay,
                    new Point(0, roiBottom),
                    new Point(frameWidth, frameHeight),
                    darkColor,
                    -1);
        }

        // Left region
        if (roiLeft > 0) {
            Imgproc.rectangle(overlay,
                    new Point(0, roiTop),
                    new Point(roiLeft, roiBottom),
                    darkColor,
                    -1);
        }

        // Right region
        if (roiRight < frameWidth) {
            Imgproc.rectangle(overlay,
                    new Point(roiRight, roiTop),
                    new Point(frameWidth, roiBottom),
                    darkColor,
                    -1);
        }

        // Blend overlay with original frame (30% opacity)
        org.opencv.core.Core.addWeighted(overlay, 0.3, frame, 0.7, 0, frame);
    }

    /**
     * Draw corner markers for better ROI visibility
     */
    private void drawCornerMarkers(Mat frame, int left, int top, int right, int bottom) {
        int markerLength = 20;
        int thickness = 3;

        // Top-left corner
        Imgproc.line(frame, new Point(left, top), new Point(left + markerLength, top), ROI_COLOR, thickness);
        Imgproc.line(frame, new Point(left, top), new Point(left, top + markerLength), ROI_COLOR, thickness);

        // Top-right corner
        Imgproc.line(frame, new Point(right, top), new Point(right - markerLength, top), ROI_COLOR, thickness);
        Imgproc.line(frame, new Point(right, top), new Point(right, top + markerLength), ROI_COLOR, thickness);

        // Bottom-left corner
        Imgproc.line(frame, new Point(left, bottom), new Point(left + markerLength, bottom), ROI_COLOR, thickness);
        Imgproc.line(frame, new Point(left, bottom), new Point(left, bottom - markerLength), ROI_COLOR, thickness);

        // Bottom-right corner
        Imgproc.line(frame, new Point(right, bottom), new Point(right - markerLength, bottom), ROI_COLOR, thickness);
        Imgproc.line(frame, new Point(right, bottom), new Point(right, bottom - markerLength), ROI_COLOR, thickness);
    }

    /**
     * Draw crosshair at ROI center
     */
    private void drawCrosshair(Mat frame, int centerX, int centerY) {
        int crosshairSize = 15;
        Scalar crosshairColor = new Scalar(0, 255, 0);  // Green

        // Horizontal line
        Imgproc.line(frame,
                new Point(centerX - crosshairSize, centerY),
                new Point(centerX + crosshairSize, centerY),
                crosshairColor,
                2);

        // Vertical line
        Imgproc.line(frame,
                new Point(centerX, centerY - crosshairSize),
                new Point(centerX, centerY + crosshairSize),
                crosshairColor,
                2);

        // Center dot
        Imgproc.circle(frame, new Point(centerX, centerY), 3, crosshairColor, -1);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Not used - all drawing is done in processFrame
    }
}