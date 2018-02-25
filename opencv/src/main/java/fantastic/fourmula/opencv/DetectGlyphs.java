package fantastic.fourmula.opencv;

import android.graphics.Color;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DetectGlyphs {
    public DetectGlyphs() {
    }

    public Mat effect(Mat input) {
        Mat gray = input;
        int a=1;
        int b=8;
        int c=150;
        int d=50;
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_RGB2GRAY);
//        Imgproc.medianBlur(gray,gray,10);
//        Imgproc.GaussianBlur(gray,gray,new Size(3,3),2);
//        gray.setTo(new Scalar(0, 0, 0, 255), gray);
        Core.convertScaleAbs(gray, gray, 1./a, 0);
        Imgproc.blur(gray, gray, new Size(b, b));
//                        Imgproc.Canny(gray, gray, 10, 20);
        Core.inRange(gray, new Scalar(d, d, d), new Scalar(c, c, c), gray);

        Core.convertScaleAbs(gray, gray, a, 0);
//        Core.bitwise_not(gray, gray);

//                Imgproc.Canny(gray, gray, 10, 20);

//        Imgproc.resize(gray, gray, gray.size(), 0.1, 0.1, Imgproc.INTER_NEAREST);
//        Imgproc.resize(gray, gray, gray.size(), 0., 0., Imgproc.INTER_NEAREST);
        //        Imgproc.Canny(gray, gray, 60, 0);
        return gray;
    }

    public ArrayList<Glyph> detect(Mat m) {
        ArrayList<Glyph> glyphs = new ArrayList<>();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(m, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        ArrayList<Detectable> detectedThings = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(i).toArray());
            double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
//            approxDistance=1;
            Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);
            MatOfPoint points = new MatOfPoint(approxCurve.toArray());
            Rect rect = Imgproc.boundingRect(points);
            Detectable d = new Detectable();
            d.currentHeight = rect.height;
            d.currentWidth = rect.width;
            d.x = rect.x;
            d.y = rect.y;

            detectedThings.add(d);
        }
        ArrayList<Area> filledAreas = new ArrayList<>();
        for (int a = 0; a < detectedThings.size(); a++) {
            Detectable d = detectedThings.get(a);
            if ((d.currentWidth >= Glyph.minimalWidth && d.currentHeight >= Glyph.minimalHeight && lim(d.currentWidth / d.currentHeight))) {
                Area currentArea = new Area(d.x, d.y, d.x + d.currentWidth, d.y + d.currentHeight);
                boolean isInterfering = false;
                for (int ar = 0; ar < filledAreas.size(); ar++) {
                    if(filledAreas.get(ar).overlaying(currentArea)){
                        isInterfering=true;
                        break;
                    }
                }
                if (!isInterfering) {
                    filledAreas.add(currentArea);
                    Glyph g = new Glyph(d.x, d.y, d.currentWidth, d.currentHeight);
                    glyphs.add(g);
                    Imgproc.rectangle(m, new Point(g.x, g.y), new Point(g.x + g.currentWidth, g.y + g.currentHeight), new Scalar(255, 0, 255), 3);
                }
            }
        }
        return glyphs;
    }

    private boolean lim(double x) {
        return (x >= 0.7 && x <= 1.3);
    }

    public static class Glyph extends Detectable {
        static final double minimalHeight = 90;
        static final double minimalWidth = 90;
        static final double maximalHeight = 500;
        static final double maximalWidth = 500;

        public Glyph(double x, double y, double width, double height) {
            this.x = x;
            this.y = y;
            this.currentWidth = width;
            this.currentHeight = height;
        }
    }

    public static class Area {
        double sx, sy, ex, ey;

        public Area(double startX, double startY, double endX, double endY) {
            sx = startX;
            sy = startY;
            ex = endX;
            ey = endY;
        }

        public boolean overlaying(Area a){
            boolean xO=false;
            boolean yO=false;
            for(double x=sx;x<=ex;x++){
                if(x<=a.ex&&x>=a.sx){
                    xO=true;
                    break;
                }
            }
            for(double y=sy;y<=ey;y++){
                if(y<=a.ey&&y>=a.sy){
                    yO=true;
                    break;
                }
            }
            return (xO&&yO);
        }

        public boolean isIncluded(double sx, double sy) {
            return sx >= this.sx && sx <= this.ex && sy >= this.sy && sy <= this.ey;
        }

        public boolean isOverlayed(double ex, double ey) {
            return ex <= this.ex && ex >= this.sx && ey <= this.ey && ey >= this.sy;
        }

        public boolean isBiggerOverlayed(double sx, double sy, double ex, double ey) {
            if (isIncluded(sx, sy)) {
                if ((ex - sx) >= (ey - ex)) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }

        public boolean isBiggerOverlayed(Area a) {
            double sx = a.sx;
            double sy = a.sy;
            double ex = a.ex;
            double ey = a.ey;
            if (isIncluded(sx, sy)) {
                if ((ex - sx) >= (ey - ex)) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
    }
}
