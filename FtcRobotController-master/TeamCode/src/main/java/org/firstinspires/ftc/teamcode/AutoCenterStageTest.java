package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoCenterStageTest extends LinearOpMode {
    OpenCvWebcam webcam;
    CenterStageExample.CenterStagePipeline pipeline;
    CenterStageExample.CenterStagePipeline.TeamPropPosition position;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CenterStageExample.CenterStagePipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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
        position = pipeline.getTeamPropPosition();
        telemetry.addData("Post-Start Position: ", position);
        telemetry.update();
        while(opModeIsActive()){
            switch (position){
                case LEFT: {

                    break;
                }
                case MID: {

                    break;
                }
                case RIGHT: {

                    break;
                }
            }
            break;
        }
        telemetry.clear();
        telemetry.addData("Post-Start Position: ", position);
        telemetry.update();
    }
}
