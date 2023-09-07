package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

import android.annotation.SuppressLint;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Concept: AprilTag", group = "Concept")
public class AprilTagOdometry extends LinearOpMode {
    // All units in metres or radians
    double FIELD_LENGTH = 3.58;
    double CAMERA_HEIGHT = 0.313;
    double YAW_ANGLE = Math.PI / 2;

    AprilTagProcessor processor;
    TfodProcessor tfProcessor;

    VisionPortal portal;
    AprilTagLocations locations;
    int[] idList = {1, 3, 2, 0};

    Vector<Double>[] positionList;
    Vector<Double>[] rotationList;

    Transform[] transformList;
    Transform currentTransform;


    // Processor implements the EOCV pipeline that processes the frames, the builder builds it with custom parameters,
    // and the vision portal connects that pipeline to the camera and hardware

    public void runOpMode() {
        // Four sides of the field
        for (int a = 0; a < positionList.length; a++) {
            positionList[a] = new Vector<>();
            rotationList[a] = new Vector<>();
            Collections.addAll(positionList[a], Math.sin(YAW_ANGLE * a) * FIELD_LENGTH / 2, -Math.cos(YAW_ANGLE * a) * FIELD_LENGTH / 2,CAMERA_HEIGHT);
            Collections.addAll(rotationList[a], (double) 0, (double) 0, YAW_ANGLE * a);
        }

        for (int x = 0; x < positionList.length; x++) {
            transformList[x] = new Transform(positionList[x], rotationList[x]);
        }

        AprilTagProcessor.Builder builder;
        VisionPortal.Builder portalBuilder = new VisionPortal.Builder();

        locations = new AprilTagLocations(idList, transformList);

        // Sets the tag library for the current season
        builder = new AprilTagProcessor.Builder();
        // TODO: This may be in inches, change to metric later
        builder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Optional: set other custom features of the AprilTag Processor (4 are shown here).
        builder.setDrawTagID(true);       // Default: true, for all detections.
        builder.setDrawTagOutline(true);  // Default: true, when tag size was provided (thus eligible for pose estimation).
        builder.setDrawAxes(true);        // Default: false.
        builder.setDrawCubeProjection(true);        // Default: false.

        processor = builder.build();
        tfProcessor = TfodProcessor.easyCreateWithDefaults();

        portalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        portalBuilder.addProcessor(processor);
        portalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        portalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        portalBuilder.enableCameraMonitoring(true);      // Enable LiveView (RC preview).
        portalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

        portal = portalBuilder.build();

        telemetry.addLine("Processor and portal have been initialised");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                List<AprilTagDetection> myAprilTagDetections = processor.getDetections();
                List<Transform> transforms = new ArrayList<>();
                for (AprilTagDetection detection : myAprilTagDetections) {
                    if (detection.metadata != null) {
                        transforms.add(TagOdometry(detection));
                    }
                }
                currentTransform = Transform.smoothAvg(transforms);
                TransformTelemetry(currentTransform);
            }
        }
    }

    public Transform TagOdometry(AprilTagDetection detection) {
        Transform currentTransform = locations.locate(detection.id);

        Vector<Double> relPos = new Vector<>();
        Vector<Double> relRot = new Vector<>();

        Collections.addAll(relPos, (double) 0, detection.ftcPose.range, (double) 0);
        // Negative signs because it is from the tag to the camera
        Collections.addAll(relRot, -detection.ftcPose.roll, -detection.ftcPose.pitch, 180-detection.ftcPose.yaw);

        Transform.Matrix rotationFromTagToCamera = Transform.Matrix.multiply(
                Transform.Matrix.rotationMatrix(detection.ftcPose.pitch - detection.ftcPose.elevation, 0, 0),
                Transform.Matrix.rotationMatrix(detection.ftcPose.bearing - detection.ftcPose.yaw, 0, 0)
        );

        // This relative position is from the april tag to the camera when the April tag is facing forwards (the y-axis) and at the origin
        relPos = Transform.Matrix.apply(rotationFromTagToCamera, relPos);

        Vector<Double> realPos = new Vector<>();
        Vector<Double> realRot = new Vector<>();

        Transform.Matrix rotationFromTagToField = Transform.Matrix.rotationMatrix(currentTransform.rot.get(0), currentTransform.rot.get(1), currentTransform.rot.get(2));

        realPos = Transform.Matrix.apply(rotationFromTagToField, relPos);
        realPos = Transform.add(realPos, currentTransform.pos);

        // Note: This is probably not accurate, as rotations are non-commutative and need special calculations, will test later
        realRot = Transform.add(relRot, currentTransform.rot);

        return new Transform(realPos, realRot);
    }

    @SuppressLint("DefaultLocale")
    public void TagTelemetry(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        telemetry.addLine(String.format("X: %.2f m, Y: %.2f m, Z: %.2f m", pose.x, pose.y, pose.z));
        telemetry.addLine(String.format("Roll: %.2f rad, Pitch: %.2f rad, Yaw: %.2f rad", pose.roll, pose.pitch, pose.yaw));
        telemetry.addLine(String.format("Range: %.2f m, Bearing: %.2f rad, Elevation: %.2f rad", pose.range, pose.bearing, pose.elevation));
        telemetry.update();
    }

    @SuppressLint("DefaultLocale")
    public void TransformTelemetry(Transform t) {
        telemetry.addLine(String.format("X: %.2f m, Y: %.2f m, Z: %.2f m", t.pos.get(0), t.pos.get(1), t.pos.get(2)));
        telemetry.addLine(String.format("Roll: %.2f rad, Pitch: %.2f rad, Yaw: %.2f rad", t.rot.get(0), t.rot.get(1), t.rot.get(2)));
        telemetry.update();
    }

    public static class AprilTagLocations {
        public int[] idArray;
        public Transform[] transformArray;
        public int l;
        public AprilTagLocations(int[] id, Transform[] transforms) {
            // Ensuring all arrays are of the same size
            if (id.length == transforms.length) {
                this.idArray = id;
                this.transformArray = transforms;
                this.l = id.length;
            } else {
                throw new IllegalArgumentException();
            }
        }

        // Returns the transform of a particular April Tag with an id
        public Transform locate(int id) {
            for (int i = 0; i < l; i++) {
                if (idArray[i] == id) {
                    return transformArray[i];
                }
            }
            return null;
        }

        // Returns the closest tag to a specified location in real space

        public int closestTag(Transform t) {
            double minDistance = 0;
            int minIndex = -1;
            for (int i = 0; i < l; i++) {
                double currentDistance = Transform.distance(t, transformArray[i]);
                if (minIndex == -1 || currentDistance < minDistance) {
                    minDistance = currentDistance;
                    minIndex = i;
                }
            }
            return idArray[minIndex];
        }
    }

    public static class Transform {
        public Vector<Double> pos;
        public Vector<Double> rot;
        public int dim;
        public Transform(Vector<Double> p, Vector<Double> r) {
            if (p.size() == r.size()) {
                this.pos = p;
                this.rot = r;
                this.dim = p.size();
            } else {
                throw new IllegalArgumentException();
            }
        }

        // Calculates Euclidean distance for any two transforms between the one which the function is
        // called on, as well as another one as an argument
        public static double distance(Transform t1, Transform t2) {
            if (t1.dim != t2.dim) {
                // Not of right dimensions
                return 0;
            }
            float diffSum = 0;
            for (int i = 0; i < t1.dim; i++) {
                diffSum += Math.pow(t1.pos.get(i) - t2.pos.get(i),2);
            }
            return Math.sqrt(diffSum);
        }
        // Calculates v1 + v2

        public static Vector<Double> add(Vector<Double> v1, Vector<Double> v2) {
            if (v1.size() != v2.size()) {
                throw new IllegalArgumentException();
            }
            Vector<Double> tmpV = new Vector<>();
            for (int i = 0; i < v1.size(); i++) {
                tmpV.add(v1.get(i) + v2.get(i));
            }
            return tmpV;
        }


        // Calculates v1 - v2
        public static Vector<Double> subtract(Vector<Double> v1, Vector<Double> v2) {
            if (v1.size() != v2.size()) {
                throw new IllegalArgumentException();
            }
            Vector<Double> tmpV = new Vector<>();
            for (int i = 0; i < v1.size(); i++) {
                tmpV.add( v1.get(i) - v2.get(i));
            }
            return tmpV;
        }

        public static Vector<Double> scale(Vector<Double> v, double c) {
            Vector<Double> tmpV = new Vector<>();
            for (int i = 0; i < v.size(); i++) {
                tmpV.add(v.get(i) * c);
            }
            return tmpV;
        }

        public static Transform smoothAvg(List<Transform> arr) {
            Vector<Double> zeroVector = new Vector<>();
            Collections.addAll(zeroVector, (double) 0, (double) 0, (double) 0);
            Transform currentTransform = new Transform(zeroVector, zeroVector);
            for (Transform t : arr) {
                currentTransform.pos = Transform.add(currentTransform.pos, Transform.scale(t.pos, 1 / (double) arr.size()));
                currentTransform.rot = Transform.add(currentTransform.rot, Transform.scale(t.rot, 1 / (double) arr.size()));
            }
            return currentTransform;
        }

        public static class Matrix {
            double[][] values;
            int rows;
            int columns;

            public Matrix(double [][] data) {
                int dim = data[0].length;
                for (double[] value : data) {
                    if (value.length != dim) {
                        // Non-contiguous matrix
                        throw new IllegalArgumentException();
                    }
                }
                this.values = data;
                this.rows = data.length;
                this.columns = dim;
            }

            public static Matrix add(Matrix m1, Matrix m2) {
                if (m1.rows == m2.rows && m1.columns == m2.columns) {
                    double[][] tmpValues = new double[m1.rows][m1.columns];
                    for (int i = 0; i < m1.rows; i++) {
                        for (int j = 0; j < m1.columns; j++) {
                            tmpValues[i][j] = m1.values[i][j] + m2.values[i][j];
                        }
                    }
                    return new Matrix(tmpValues);
                } else {
                    throw new IllegalArgumentException();
                }
            }

            public static Matrix scale(Matrix m, double f) {
                double[][] tmpValues = new double[m.rows][m.columns];
                for (int i = 0; i < m.rows; i++) {
                    for (int j = 0; j < m.columns; j++) {
                        tmpValues[i][j] = m.values[i][j] * f;
                    }
                }
                return new Matrix(tmpValues);
            }
            public static Vector<Double> apply(Matrix m, Vector<Double> v) {
                if (v.size() == m.columns) {
                    Vector<Double> tmpV = new Vector<>();
                    for (int i = 0; i < m.rows; i++) {
                        double sum = 0;
                        for (int j = 0; j < v.size(); j++) {
                            sum += m.values[i][j] * v.get(j);
                        }
                        tmpV.add(sum);
                    }
                    return tmpV;
                } else {
                    throw new IllegalArgumentException();
                }
            }

            // This performs m1 * m2
            public static Matrix multiply(Matrix m1, Matrix m2) {
                if (m1.columns == m2.rows) {
                    double[][] tmpValues = new double[m1.rows][m2.columns];
                    for (int i = 0; i < m1.rows; i++) {
                        for (int j = 0; j < m2.columns; j++) {
                            double sum = 0;
                            for (int k = 0; k < m1.columns; k++) {
                                sum += m1.values[i][k] * m2.values[k][j];
                            }
                            tmpValues[i][j] = sum;
                        }
                    }
                    return new Matrix(tmpValues);
                } else {
                    throw new IllegalArgumentException();
                }
            }

            // This performs it element wise
            public static Matrix dot(Matrix m1, Matrix m2) {
                if (m1.rows == m2.rows && m1.columns == m2.columns) {
                    double[][] tmpValues = new double[m1.rows][m1.columns];
                    for (int i = 0; i < m1.rows; i++) {
                        for (int j = 0; j < m1.columns; j++) {
                            tmpValues[i][j] = m1.values[i][j] * m2.values[i][j];
                        }
                    }
                    return new Matrix(tmpValues);
                } else {
                    throw new IllegalArgumentException();
                }
            }

            public static Matrix Zeros(int s) {
                double[][] tmpValues = new double[s][s];
                for (int i = 0; i < s; i++) {
                    for (int j = 0; j < s; j++) {
                        tmpValues[i][j] = 0;
                    }
                }
                return new Matrix(tmpValues);
            }

            public static Matrix Ones(int s) {
                double[][] tmpValues = new double[s][s];
                for (int i = 0; i < s; i++) {
                    for (int j = 0; j < s; j++) {
                        tmpValues[i][j] = 1;
                    }
                }
                return new Matrix(tmpValues);
            }

            public static Matrix Identity(int s) {
                double[][] tmpValues = new double[s][s];
                for (int i = 0; i < s; i++) {
                    for (int j = 0; j < s; j++) {
                        if (i != j) {
                            tmpValues[i][j] = 0;
                        } else {
                            tmpValues[i][j] = 1;
                        }
                    }
                }
                return new Matrix(tmpValues);
            }

            // Note, angles are in radians here and will go from x to y to z (roll, pitch, yaw)
            public static Matrix rotationMatrix(double...angles) {
                Matrix rollMatrix = new Matrix(new double[][]
                        {
                                {1, 0, 0},
                                {0, Math.cos(angles[0]), -Math.sin(angles[0])},
                                {0, Math.sin(angles[0]), Math.cos(angles[0])}
                        }
                );
                Matrix pitchMatrix = new Matrix(new double[][]
                        {
                                {Math.cos(angles[0]), 0, Math.sin(angles[0])},
                                {0, 1, 0},
                                {-Math.sin(angles[0]), 0, Math.cos(angles[0])}
                        }
                );
                Matrix yawMatrix = new Matrix(new double[][]
                        {
                                {Math.cos(angles[0]), -Math.sin(angles[0]), 0},
                                {Math.sin(angles[0]), Math.cos(angles[0]), 0},
                                {0, 0, 1}
                        }
                );
                return Matrix.multiply(yawMatrix, Matrix.multiply(pitchMatrix, rollMatrix));
            }
        }
    }
}