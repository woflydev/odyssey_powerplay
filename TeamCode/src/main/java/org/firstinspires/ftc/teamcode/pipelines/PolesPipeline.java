package org.firstinspires.ftc.teamcode.pipelines;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.BFMatcher;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;


public class PolesPipeline extends OpenCvPipeline {
    private final Scalar RED = new Scalar(0, 0, 255);
    private final Scalar GREEN = new Scalar(0, 255, 0);
    private final Scalar YELLOW = new Scalar(0, 255, 255);
    private final Scalar BLUE = new Scalar(255, 0, 0);
    private final Scalar PINK = new Scalar(255, 0, 255);
    private final Scalar CYAN = new Scalar(255, 255, 0);
    private final Scalar WHITE = new Scalar(255, 255, 255);

    private final int GRID_SIZE = 180;
    private final int[][] TRUE_PTS = {
            {-1, -2}, {1, -2},
            {-2, -1}, {-1, -1}, {0, -1}, {1, -1}, {2, -1},
            {-1, 0}, {1, 0},
            {-2, 1}, {-1, 1}, {0, 1}, {1, 1}, {2, 1},
            {-1, 2}, {1, 2}
    };
    private final double[] START_POS = {1.3, 1.7};
    private final double[] CORNER_POS = {3, 3};
    private final int MAP_SIZE = 700;

    private final Scalar LOWER_BOUND = new Scalar(new double[] {10, 120, 50});
    private final Scalar UPPER_BOUND = new Scalar(new double[] {30, 255, 255});
    private final int W = 1300;

    private final double RUNTIME = 120;
    private final double AREA_THRESHOLD = 100;



    private Scalar RandomColor() {
        Random r = new Random();
        int[] i = r.ints(3, 0, 255).toArray();
        return new Scalar(i[0], i[1], i[2]);
    }

    private Mat RotationMatrix(double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        Mat mat = new Mat(2, 2, CvType.CV_64F);
        mat.put(new int[] {2, 2}, c, -s, s, c);
        return mat;
    }

    private Mat TranslationMatrix(double[] v) {
        Mat mat = new Mat(2, 3, CvType.CV_64F);
        mat.put(new int[] {2, 3}, 1, 0, v[0], 0, 1, v[1]);
        return mat;
    }

    private Point IntersectionPoint(Point a1, Point a2, Point b1, Point b2) {
        double m1 = (a2.y - a1.y) / (a2.x - a1.x);
        double c1 = a2.y - m1 * a2.x;
        double m2 = (b2.y - b1.y) / (b2.x - b1.x);
        double c2 = b2.y - m2 * b2.x;

        double x = -(c2 - c1) / (m2 - m1);
        Point inter = new Point(x, m1 * x + c1);

        return inter;
    }

    // O(n^2) complexity, had to do for convenience
    // cmpFn takes in an argument of type T and outputs a value which will be used for comparison (sorts in ascending order)
    private static <T> List<T> CustomSort(List<T> src, Function<T, java.lang.Double> cmpFn) {
        List<java.lang.Integer> indexList = new ArrayList<>();
        for (int i = 0; i < src.size(); i++) {
            double currentMin = 0;
            int minIndex = 0;
            boolean firstComparison = true;
            for (int j = 0; j < src.size(); j++) {
                if (!indexList.contains(j)) {
                    if (firstComparison) {
                        currentMin = cmpFn.apply(src.get(j));
                        minIndex = j;
                    } else {
                        double testVal = cmpFn.apply(src.get(j));
                        if (testVal < currentMin) {
                            currentMin = testVal;
                            minIndex = j;
                        }
                    }
                }
            }
            indexList.add(minIndex);
        }

        List<T> destArr = new ArrayList<>();
        for (int i = 0; i < src.size(); i++) {
            destArr.add(src.get(indexList.get(i)));
        }

        return destArr;
    }

    private double [] oldPos = START_POS;
    private Mat oldMat = TranslationMatrix(START_POS);
    private Mat mapImg = Mat.zeros(new int[] {MAP_SIZE, MAP_SIZE, 3}, CvType.CV_8U);
    BFMatcher matcher = new BFMatcher(Core.NORM_L2, true);


    private Mat hsv = new Mat();
    private Mat mask = new Mat();

    private List<java.lang.Double> newPts = new ArrayList<>();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    private MatOfInt hull = new MatOfInt();
    private MatOfPoint2f hullPts = new MatOfPoint2f();
    private RotatedRect rect = new RotatedRect();

    private Mat boxPoints = new Mat();

    private Mat line = new Mat();

    public ElapsedTime timer = new ElapsedTime();
    public double startTime;

    @Override
    public void init(Mat input) {
        startTime = timer.seconds();
    }
    @Override
    public Mat processFrame(Mat input) {
        // Arbitrary, I don't know what is happening here
        if (timer.seconds() - startTime < RUNTIME) {
            hsv.release();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
            mask.release();
            Core.inRange(hsv, LOWER_BOUND, UPPER_BOUND, mask);
            int X = Math.round((float)input.size().width), Y = Math.round((float)input.size().height);
            int yf = (int) Math.round(Y * 1.3);

            Mat srcPlane = new Mat(4, 2, CvType.CV_8U);
            Mat destPlane = new Mat(4, 2, CvType.CV_8U);
            srcPlane.put(new int[] {4, 2}, new int[] {0, 0, X, 0, X+W, Y, -W, Y});
            destPlane.put(new int[] {4, 2}, new int[] {0, 0, X, 0, X, yf, 0, yf});

            Mat perspMat = Imgproc.getPerspectiveTransform(srcPlane, destPlane);
            Mat warpedImg = new Mat();
            Imgproc.warpPerspective(input, warpedImg, perspMat, new Size(X, yf));

            newPts = new ArrayList<>();
            hierarchy.release();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > AREA_THRESHOLD) {
                    hull.release();
                    Imgproc.convexHull(contour, hull);

                    hullPts.create(hull.size(), CvType.CV_32F);
                    List<Point> tmpPoints = new ArrayList<>();
                    for (int e : hull.toArray()) {
                        double tmp[] = contour.get(0, e);
                        tmpPoints.add(new Point(tmp));
                    }

                    hullPts.fromList(tmpPoints);

                    List<MatOfPoint> hullPtsWrapper = new ArrayList<>();
                    hullPtsWrapper.add(new MatOfPoint(hullPts));

                    // Draw contours here
                    Imgproc.drawContours(input, hullPtsWrapper, -1, GREEN, 1);

                    rect = Imgproc.minAreaRect(hullPts);

                    Imgproc.boxPoints(rect, boxPoints);
                    List<MatOfPoint> boxPtsWrapper = new ArrayList<>();
                    boxPtsWrapper.add(new MatOfPoint(boxPoints));
                    Imgproc.drawContours(input, boxPtsWrapper, -1, GREEN, 1);


                    // Find centroid
                    Moments M = Imgproc.moments(contour);
                    Point centre = new Point(M.m10 / M.m00, M.m01 / M.m00);

                    Imgproc.fitLine(hull, line, Imgproc.DIST_L2, 0, 0.01, 0.01);
                    double lineSegment[] = new double[4];
                    line.get(0, 0, lineSegment);
                    double vx = lineSegment[0], vy = lineSegment[1], x = lineSegment[2], y = lineSegment[3];

                    int[] tmpBoxPoints = new int[(int)boxPoints.total() * boxPoints.channels()];
                    boxPoints.get(0, 0, tmpBoxPoints);
                    List<Point> boxPointsArr = new ArrayList<>();
                    for (int i = 0; i < tmpBoxPoints.length / 2; i++) {
                        boxPointsArr.add(new Point(tmpBoxPoints[2 * i], tmpBoxPoints[2 * i + 1]));
                    }

                    boxPointsArr = CustomSort(boxPointsArr, (Point p) -> p.y);
                    Point left = boxPointsArr.get(boxPointsArr.size() - 2);
                    Point right = boxPointsArr.get(boxPointsArr.size() - 1);

                    Point base = IntersectionPoint(new Point(x, y), new Point(x + vx, x+ vy),
                            left, right);

                    Mat featurePlane = new Mat(4, 2, CvType.CV_8U);
                    featurePlane.put(4,2, base.x, base.y, left.x, left.y, right.x, right.y, centre.x, centre.y);

                    Mat warpedFeatures = new Mat();
                    // Goes in order, base, left, right, centre
                    Imgproc.warpPerspective(featurePlane, warpedFeatures, perspMat, new Size(X, yf));

                    Point relPos = new Point((warpedFeatures.get(0, 0)[0] - X / 2) / GRID_SIZE,
                            (warpedFeatures.get(0, 1)[0] - yf) / GRID_SIZE);

                    Mat dotMat = new Mat(1, 3, CvType.CV_32F);
                    dotMat.put(1, 3, relPos.x, relPos.y, 1);

                    double newPt = oldMat.dot(dotMat);
                    newPts.add(newPt);

                    Imgproc.line(input, base, centre, BLUE, 1);
                    // Continue from there

                    featurePlane.release();
                    warpedFeatures.release();
                }
            }
            srcPlane.release();
            destPlane.release();
            perspMat.release();
            warpedImg.release();
        }

        return input;

    }

    @Override
    public void onViewportTapped() {

    }
}
