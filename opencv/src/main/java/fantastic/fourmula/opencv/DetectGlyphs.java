package fantastic.fourmula.opencv;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class DetectGlyphs {
    public DetectGlyphs(){}

    public Mat effect(CameraBridgeViewBase.CvCameraViewFrame input){
        Mat gray=input.rgba();
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(gray,gray,100,100);
        return gray;
    }
}
