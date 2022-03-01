/*
 * Copyright (c) 2020 OpenFTC Team
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous
public class easyautonomous extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    /* Declare OpMode members. */
    HardwareRovRuk_1 robot = new HardwareRovRuk_1();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;                   // sets rate to move servo

    static final double COUNTS_PER_MOTOR_REV = 145.1;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.25;
    static final double STRAFE_SPEED = 0.4;

    static final double COUNTS_PER_MOTOR_REV_1620 = 103.8;    // eg: gobilda 5202 series 1620rpm Motor Encoder
    static final double WORM_GEAR_REDUCTION = 24.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV_1620 * WORM_GEAR_REDUCTION / 360; //counts per degree rotation on worm
    //gear output shaft

    @Override
    public void runOpMode() {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        robot.FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path0", "Starting at %7d :%7d",
//                robot.FrontLeftDrive.getCurrentPosition(),
//                robot.RearLeftDrive.getCurrentPosition(),
//                robot.FrontRightDrive.getCurrentPosition(),
//                robot.RearRightDrive.getCurrentPosition());
//        telemetry.update();


        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        //robot.Carm.setPosition(0.75);

        //encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 5);
        //encoderDrive(DRIVE_SPEED, -12, -12, 12, 12, 5);
        //encoderDrive(DRIVE_SPEED, 6, -6, -6, 6, 5);
        encoderDrive(DRIVE_SPEED, 24, 24, 24, 24, 5);
        /*
        do {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.update();

            if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.LEFT) {

                // 0 - move Sarm out of the way
                // 1 - drop Barm (big arm / cube arm)
                // 2 - back up 1 inch
                // 3 - pivot 45 degrees (with fudge factor)
                // 4 - back up to shelf  from point (A)
                // 5 - drop cube
                // 6 - drive back to point (A)
                // 7 - turn left 45 deg
                // 8 - drive forward

                moveSarm(-90, 4);
                moveBarm(-270, 4);

                //encoderDrive(DRIVE_SPEED, -5, -5, -5, -5, 4);
                encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);
                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 4);
                robot.Carm.setPosition(0.3); // opens
                //encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 4);

                //moveBarm(-250, 1);

                //encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);

                //encoderDrive(DRIVE_SPEED, 24, -24, -24, 24, 4);

                telemetry.addData("found position", "Left");
                telemetry.addData("movement", "forward");
                break;
            } else if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.CENTER) {
                // 0 - move Sarm out of the way
                // 1 - drop Barm (big arm / cube arm)
                // 2 - back up 1 inch
                // 3 - pivot 45 degrees (with fudge factor)
                // 4 - back up to shelf  from point (A)
                // 5 - drop cube
                // 6 - drive back to point (A)
                // 7 - turn left 45 deg
                // 8 - drive forward

                moveSarm(-100, 4);
                moveBarm(-180, 4);

                encoderDrive(DRIVE_SPEED, -1, -1, -1, -1, 4);
                encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);
                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 4);
                robot.Carm.setPosition(0.3); // opens
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 4);

                //moveBarm(-250, 1);

                encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);

                encoderDrive(DRIVE_SPEED, 24, -24, -24, 24, 4);

                telemetry.addData("found position", "Center");
                telemetry.addData("movement", "forward");
                break;

            } else if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.RIGHT) {
                // 0 - move Sarm out of the way
                // 1 - drop Barm (big arm / cube arm)
                // 2 - back up 1 inch
                // 3 - pivot 45 degrees (with fudge factor)
                // 4 - back up to shelf  from point (A)
                // 5 - drop cube
                // 6 - drive back to point (A)
                // 7 - turn left 45 deg
                // 8 - drive forward

                moveSarm(-100, 4);
                moveBarm(-270, 4);

                encoderDrive(DRIVE_SPEED, -1, -1, -1, -1, 4);
                encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);
                encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 4);
                robot.Carm.setPosition(0.3); // opens
                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 4);

                //moveBarm(-250, 1);

                encoderDrive(DRIVE_SPEED, -18, -18, 0, 0, 4);

                encoderDrive(DRIVE_SPEED, 24, -24, -24, 24, 4);

                telemetry.addData("found position", "Right");
                telemetry.addData("movement", "forward");
                break;

            }

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        } while (false);*/
    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109, 98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253, 98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
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

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile SkystonePosition position = SkystonePosition.LEFT;

        // changed to public for testing calling position above
        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame) {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if (max == avg1) // Was it from region 1?
            {
                position = SkystonePosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg2) // Was it from region 2?
            {
                position = SkystonePosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (max == avg3) // Was it from region 3?
            {
                position = SkystonePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SkystonePosition getAnalysis() {
            return position;
        }
    }

    public void encoderDrive(double speed,
                             double FrontLeftInches, double RearLeftInches, double FrontRightInches, double RearRightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.FrontLeftDrive.getCurrentPosition() + (int) (FrontLeftInches * COUNTS_PER_INCH);
            newRearLeftTarget = robot.RearLeftDrive.getCurrentPosition() + (int) (RearLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robot.FrontRightDrive.getCurrentPosition() + (int) (FrontRightInches * COUNTS_PER_INCH);
            newRearRightTarget = robot.RearRightDrive.getCurrentPosition() + (int) (RearRightInches * COUNTS_PER_INCH);
            robot.FrontLeftDrive.setTargetPosition(newFrontLeftTarget);
            robot.RearLeftDrive.setTargetPosition(newRearLeftTarget);
            robot.FrontRightDrive.setTargetPosition(newFrontRightTarget);
            robot.RearRightDrive.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FrontLeftDrive.setPower(Math.abs(speed));
            robot.RearLeftDrive.setPower(Math.abs(speed));
            robot.FrontRightDrive.setPower(Math.abs(speed));
            robot.RearRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() <
                            timeoutS) &&
                    (robot.FrontLeftDrive.isBusy() && robot.RearLeftDrive.isBusy() && robot.FrontRightDrive.isBusy() && robot.RearRightDrive.isBusy())) {

                // Display it for the driver. edited out because of error when init robot
//                telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newRearLeftTarget, newFrontRightTarget, newRearRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
//                        robot.FrontLeftDrive.getCurrentPosition(),
//                        robot.RearLeftDrive.getCurrentPosition(),
//                        robot.FrontRightDrive.getCurrentPosition(),
//                        robot.RearRightDrive.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            robot.FrontLeftDrive.setPower(0);
            robot.RearLeftDrive.setPower(0);
            robot.FrontRightDrive.setPower(0);
            robot.RearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }

    public void moveBarm(double angleInDegrees, double timeoutS) {
        int newTargetCount = 0;

        if (opModeIsActive()) {
            newTargetCount = robot.Barm.getCurrentPosition() + (int) (angleInDegrees * COUNTS_PER_DEGREE);
            robot.Barm.setTargetPosition(newTargetCount);
            robot.Barm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.Barm.setPower(Math.abs(.5));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.Barm.isBusy())) {
//                telemetry.addData("Barm:", "Running at %7d :%7d",
//                        robot.Barm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Barm.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Barm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }

    public void moveSarm(double angleInDegrees, double timeoutS) {
        int newTargetCount = 0;

        if (opModeIsActive()) {
            newTargetCount = robot.Sarm.getCurrentPosition() + (int) (angleInDegrees * COUNTS_PER_DEGREE);
            robot.Sarm.setTargetPosition(newTargetCount);
            robot.Sarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.Sarm.setPower(Math.abs(.85));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.Sarm.isBusy())) {
//                telemetry.addData("Sarm:", "Running at %7d :%7d",
//                        robot.Sarm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Sarm.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Sarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }
    }
}
