package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class CenterStageExample extends LinearOpMode{
    OpenCvWebcam webcam;
    CenterStagePipeline pipeline;
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CenterStagePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while(opModeInInit()){
            telemetry.addData("Position: ", pipeline.getTeamPropPosition());
            telemetry.update();
            sleep(50);
        }
        while(opModeIsActive()){

        }
    }
    public static class CenterStagePipeline extends OpenCvPipeline {
        public enum TeamPropPosition
        {
            LEFT,
            MID,
            RIGHT,
        }
        private volatile TeamPropPosition position = TeamPropPosition.LEFT;
        static final Scalar BLUE = new Scalar(0,0,255);
        static final Scalar GREEN = new Scalar(0,255,0);
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_RGB, region2_RGB, region3_RGB;
        Mat RGBA = new Mat();
        Mat R = new Mat();
        int avg1, avg2, avg3;

        void inputToRGB(Mat input){
            Imgproc.cvtColor(input, RGBA,Imgproc.COLOR_RGB2RGBA);
            Core.extractChannel(RGBA, R, 0);
        }
        @Override
        public void init(Mat firstFrame){
            inputToRGB(firstFrame);
            region1_RGB = R.submat(new Rect(region1_pointA, region1_pointB));
            region2_RGB = R.submat(new Rect(region2_pointA, region2_pointB));
            region3_RGB = R.submat(new Rect(region3_pointA, region3_pointB));
        }
        @Override
        public Mat processFrame(Mat input) {
            inputToRGB(input);

            avg1 = (int) Core.mean(region1_RGB).val[0];
            avg2 = (int) Core.mean(region2_RGB).val[0];
            avg3 = (int) Core.mean(region3_RGB).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    BLUE,
                    2);
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    BLUE,
                    2);
            int maxOfFirst2 = Math.max(avg1,avg2);
            int max = Math.max(maxOfFirst2,avg3);
            if(max == avg1){
                position = TeamPropPosition.LEFT;
                Imgproc.rectangle(
                        input,
                        region1_pointA,
                        region1_pointB,
                        GREEN,
                        -1
                );
            }
            else if(max == avg2){
                position = TeamPropPosition.MID;
                Imgproc.rectangle(
                        input,
                        region2_pointA,
                        region2_pointB,
                        GREEN,
                        -1
                );
            }
            else if(max == avg3){
                position = TeamPropPosition.RIGHT;
                Imgproc.rectangle(
                        input,
                        region3_pointA,
                        region3_pointB,
                        GREEN,
                        -1
                );
            }
            return input;
        }
        public TeamPropPosition getTeamPropPosition(){
            return position;
        }
    }
}
