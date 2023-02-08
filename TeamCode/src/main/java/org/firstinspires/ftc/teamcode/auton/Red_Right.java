/*
 * Copyright (c) 2021 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NM12351Hardware;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "Red_Right", group = "Group 1")
public class Red_Right extends LinearOpMode

{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    private ElapsedTime runtime = new ElapsedTime();

    private NM12351Hardware robot = new NM12351Hardware(this);

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        robot.init(hardwareMap);

        robot.rs.setPosition(0);
        robot.cs.setPosition(0);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
       /* if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        //default trajectory here if preferred
        if(tagOfInterest.id == LEFT){
            robot.strafeForCounts(75,-0.5);
            robot.driveForCounts(1700, 0.5);
            robot.NMWait(100);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(100);
            robot.turnForCounts(1000,-0.5);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(500);
            robot.liftForCounts(1000,1);
            robot.rs.setPosition(1);
            robot.NMWait(500);
            robot.flipLift();
            robot.NMWait(1000);
            robot.cs.setPosition(1);
            robot.NMWait(500);
            robot.liftDownForCounts(1250,0.75);
            robot.NMWait(500);
            robot.flipLiftBack();
            robot.NMWait(200);
            robot.rs.setPosition(0.38);
            robot.NMWait(500);
            robot.turnForCounts(330,0.5);
            robot.driveForCounts(600,-0.5);
            robot.turnForCounts(550,0.5);
            robot.strafeForCounts(0, 0.5);
            robot.driveForCounts(100,-0.5);



           /* for (int i = 0; i < 2; i++) {
                robot.strafeForCounts(325, -0.5);
                robot.turnForCounts(650, -0.5);
                robot.driveForCounts(830, 0.5);
                robot.NMWait(500);
                robot.driveForCounts(800, -0.5);
                robot.turnForCounts(600, 0.5);
                robot.strafeForCounts(345, 0.5);
                robot.NMWait(500);*/
            //    }

            // robot.strafeForCounts(1200,-0.5);

            //left trajectory
        }else if(tagOfInterest.id == MIDDLE){
            robot.strafeForCounts(75,-0.5);
            robot.driveForCounts(1700, 0.5);
            robot.NMWait(100);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(100);
            robot.turnForCounts(1000,-0.5);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(500);
            robot.liftForCounts(1000,1);
            robot.rs.setPosition(1);
            robot.NMWait(500);
            robot.flipLift();
            robot.NMWait(1000);
            robot.cs.setPosition(1);
            robot.NMWait(500);
            robot.liftDownForCounts(1250,0.75);
            robot.NMWait(500);
            robot.flipLiftBack();
            robot.NMWait(200);
            robot.rs.setPosition(0.38);
            robot.NMWait(500);
            robot.turnForCounts(250,-0.5);
            robot.driveForCounts(200,0.5);

        }else {
            robot.strafeForCounts(75,-0.5);
            robot.driveForCounts(1700, 0.5);
            robot.NMWait(100);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(100);
            robot.turnForCounts(1000,-0.5);
            robot.driveForCounts(100,-0.5);
            robot.NMWait(500);
            robot.liftForCounts(1000,1);
            robot.rs.setPosition(1);
            robot.NMWait(500);
            robot.flipLift();
            robot.NMWait(1000);
            robot.cs.setPosition(1);
            robot.NMWait(500);
            robot.liftDownForCounts(1250,0.75);
            robot.NMWait(500);
            robot.flipLiftBack();
            robot.NMWait(200);
            robot.rs.setPosition(0.38);
            robot.NMWait(500);
            robot.turnForCounts(960,-0.5);
            robot.driveForCounts(500,-0.5);
            robot.turnForCounts(730,-0.5);
            robot.strafeForCounts(100, -0.5);
            robot.driveForCounts(100,-0.5);

        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}