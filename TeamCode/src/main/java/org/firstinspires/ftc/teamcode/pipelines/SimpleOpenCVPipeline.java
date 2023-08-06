package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SimpleOpenCVPipeline extends OpenCvPipeline {
    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);

    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat maskedInputMat = new Mat();

    @Override
    public void init(Mat input) {
        // Executed before the first call to proceed

    }
    // Note to self, always define common variables at the start of the pipeline
    // otherwise memory becomes an issue, unless mat.release() is called

    // Always use .release() before overwriting old data, especially when processing
    // at high speed
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, lower, upper, binaryMat);

        maskedInputMat.release();

        Core.bitwise_and(input, input, maskedInputMat, binaryMat);
        // Input to output
        return maskedInputMat;
    }

    @Override
    public void onViewportTapped() {
        // Executed when the image display is clicked by the mouse or tapped
        // This method is executed from the UI thread, so be careful to not
        // perform any sort heavy processing here! Your app might hang otherwise
    }

}
