package org.firstinspires.ftc.teamcode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */

public class ScanColoredObjects extends OpenCVPipeline {
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private String requiredObject = new String("");
    // this is just here so we can expose it later thru getContours.
    private List<MatOfPoint> contours = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized List<MatOfPoint> getContours() {
        return contours;
    }
    public synchronized void setRequiredObject(String requiredObj){
        requiredObject  = requiredObj;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        //Convert the RGB to HSV for better accuracy
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        //Apply the filter to get the required color
        if(requiredObject.compareToIgnoreCase("zoneBorderBlue") == 0)
        {
            Core.inRange(hsv, new Scalar(90, 128, 30), new Scalar(170, 255, 255), thresholded);
        }
        else if(requiredObject.compareToIgnoreCase("zoneBorderRed") == 0){
            Core.inRange(hsv, new Scalar(300, 128, 128), new Scalar(300, 256, 256), thresholded);
        }
        else {
            Core.inRange(hsv, new Scalar(10, 220, 100), new Scalar(20, 256, 206), thresholded);
        }

        Imgproc.blur(thresholded, thresholded, new Size(3, 3));
        contours = new ArrayList<>();

        //Fill up the contour with the objects which are in required color
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //TODO Gyanesh - Temporary code to debug, remove the if condition later
        if (showContours) {
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }

        return rgba; // display the image seen by the camera
    }
}
