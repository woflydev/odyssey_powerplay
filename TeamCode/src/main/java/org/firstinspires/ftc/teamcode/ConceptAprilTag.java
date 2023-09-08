/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This 2023-2024 OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag2", group = "Concept")
public class ConceptAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static double FIELD_LENGTH = 3.58;
    private static double CAMERA_HEIGHT = 0.313;

    private static double YAW_ANGLE = Math.PI / 2;

    private static int ACQUISITION_TIME = 10;

    private static int SLEEP_TIME = 20;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // This assumes the april tag starts facing along the y-axis, may change later
    private AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(0, "Forward", 0.1,
                    new VectorF(0, (float) FIELD_LENGTH / 2, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, Quaternion.identityQuaternion()),
            new AprilTagMetadata(1, "Left", 0.1,
                    new VectorF((float) - FIELD_LENGTH / 2, 0, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                            (float) Math.cos(- YAW_ANGLE / 2), 0, 0,
                            (float) Math.sin(- YAW_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(3, "Back", 0.1,
                    new VectorF(0, (float) - FIELD_LENGTH / 2, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(YAW_ANGLE), 0, 0,
                    (float) Math.sin(YAW_ANGLE), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(2, "Right", 0.1,
                    new VectorF((float) FIELD_LENGTH / 2, 0, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(YAW_ANGLE / 2), 0, 0,
                    (float) Math.sin(YAW_ANGLE / 2), ACQUISITION_TIME))
    };

    private VectorF previousPosition;

    private VectorF currentPosition;

    private VectorF currentVelocity;

    private long blindTime = 0;
    private boolean isBlind = false;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        currentPosition = new VectorF(0, 0, (float)CAMERA_HEIGHT);
        previousPosition = new VectorF(0, 0, (float)CAMERA_HEIGHT);
        currentVelocity = new VectorF(0, 0, 0);

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(SLEEP_TIME);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (AprilTagMetadata tag : tagArray) {
            b.addTag(tag);
        }

        AprilTagLibrary library = b.build();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            //.setDrawAxes(false)
            //.setDrawCubeProjection(false)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
            .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            .setLensIntrinsics(1389.80870649, 1389.80870649, 663.268596171, 399.045042197)
            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        VectorF smoothPos = new VectorF(0, 0, 0);
        int notNullTags = 0;

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                AprilTagPoseFtc apose = detection.ftcPose;

                float[] vec = detection.metadata.fieldPosition.added(new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z)).getData();
                float angle = (float) (-detection.ftcPose.yaw);
                float[] tagPose = detection.metadata.fieldPosition.getData();
                double[] cameraPoints = rotatePoint(vec[0], vec[1], tagPose[0], tagPose[1], angle);
                telemetry.addLine(String.format("New XY rot %6.1f %6.1f %6.1f  (meter)", cameraPoints[0], cameraPoints[1], angle));
                telemetry.addLine(String.format("New XY %6.1f %6.1f  (meter)", vec[0], vec[1]));

                telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                smoothPos.add(new VectorF((float)cameraPoints[0], (float)cameraPoints[1], (float)CAMERA_HEIGHT));
                notNullTags++;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        previousPosition = currentPosition;

        if (notNullTags > 0) {
            currentPosition = smoothPos.multiplied(1 / (float) notNullTags);
            currentVelocity = currentPosition.subtracted(previousPosition).multiplied(1 / (float) SLEEP_TIME);
            isBlind = false;
        } else {
            // Assumes constant velocity if no April tags can be seen
            if (!isBlind) {
                blindTime = elapsedTime.time(timeUnit);
                isBlind = true;
            }
            currentPosition = previousPosition.added(currentVelocity.multiplied(elapsedTime.time(timeUnit) - blindTime));
        }

        //negative yaw is rotate tag to the left
        //pitch is up and down +, -
        
        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public double[] rotatePoint(double rX, double rY, double cX, double cY, double angleInDegrees)
    {
        double angleInRadians = Math.toRadians(angleInDegrees);
        double cosTheta = Math.cos(angleInRadians);
        double sinTheta = Math.sin(angleInRadians);
        return new double[]{
                (cosTheta * (rX - cX) -
                        sinTheta * (rY - cY) + cX),
                (sinTheta * (rX - cX) +
                        cosTheta * (rY - cY) + cY)};
    }
}
