package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Blue Ducks 2.1")

public class BD_OpenCV_Freight_Frenzy extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    HardwareRovRuk_1 robot = new HardwareRovRuk_1();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 145.1;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static double DRIVE_SPEED = 0.25;

    static final double COUNTS_PER_MOTOR_REV_1620 = 103.8;    // eg: gobilda 5202 series 1620rpm Motor Encoder
    static final double WORM_GEAR_REDUCTION = 24.0;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV_1620 * WORM_GEAR_REDUCTION / 360; //counts per degree rotation on worm

    //Orange
    private double crThreshHigh = 255;
    private double crThreshLow = 166;
    private double cbThreshHigh = 103;
    private double cbThreshLow = 0; //increase range

    //Green
    //private double crThreshHigh = 127;
    //private double crThreshLow = 0;
    //private double cbThreshHigh = 127;
    //private double cbThreshLow = 0; //increase range

    private int minRectangleArea = 1200;

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Orange Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 166.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 103.0);

    // Green Range
    //public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 127.0, 127.0);

    @Override
    public void runOpMode() throws InterruptedException
    {
        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Only if you are using ftcdashboard
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        robot.init(hardwareMap);

        robot.Carm.setPosition(0.3);

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

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive())
        {
            if(pipeline.error){
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }
            // Only use this line of the code when you want to find the lower and upper values, using Ftc Dashboard (https://acmerobotics.github.io/ftc-dashboard/gettingstarted)
            // testing(pipeline);

            // Watch our YouTube Tutorial for the better explanation
            double midpointX = pipeline.getRectMidpointX();
            double rectangleArea = pipeline.getRectArea();

            //Print out the area of the rectangle that is found.
            telemetry.addData("Rectangle Area", rectangleArea);
            telemetry.addData("Camera Width: ", pipeline.getRectWidth());
            telemetry.addData("Rectangle Midpoint: ", midpointX);

            robot.Carm.setPosition(0.65);

            //Check to see if the rectangle has a large enough area to be a marker.
            if(rectangleArea > minRectangleArea){
                //Then check the location of the rectangle to see which barcode it is in.
                if(midpointX > 426.0){

                    telemetry.addData("found position", "Right");

                    encoderDrive(DRIVE_SPEED, 3, -3, -3, 3, 4);
                    encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 4);
                    moveSarm(-90, 4);
                    moveBarm(-192, 4);
                    encoderDrive(DRIVE_SPEED, 23, -23, -23, 23, 6);
                    encoderDrive(DRIVE_SPEED, -7, -7, -7, -7, 6);
                    robot.Carm.setPosition(0.3); // opens
                    sleep(750);
                    robot.Carm.setPosition(0.65); // closes
                    encoderDrive(DRIVE_SPEED, 6.5, 6.5, 6.5, 6.5, 6);
                    moveBarm(188, 4);
                    robot.Warm.setPower(0.75);
                    encoderDrive(DRIVE_SPEED, -35, 35, 35, -35, 8);
                    DRIVE_SPEED = 0.1;
                    encoderDrive(DRIVE_SPEED, -9, 9, 9, -9, 4);
                    DRIVE_SPEED = 0.25;
                    sleep(2500);
                    robot.Warm.setPower(0);
                    encoderDrive(DRIVE_SPEED, -9, -9, -9, -9, 8);
                    encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 8);

                    break;
                }
                else if(midpointX < 213.0){

                    telemetry.addData("found position", "Left");

                    encoderDrive(DRIVE_SPEED, 3, -3, -3, 3, 4);
                    encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 4);
                    moveSarm(-90, 4);
                    moveBarm(-234, 4);
                    encoderDrive(DRIVE_SPEED, 20, -20, -20, 20, 6);
                    encoderDrive(DRIVE_SPEED, -4, -4, -4, -4, 6);
                    robot.Carm.setPosition(0.5); // opens
                    sleep(750);
                    encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 6);
                    moveBarm(225, 4);
                    robot.Warm.setPower(0.75);
                    DRIVE_SPEED = 0.175;
                    encoderDrive(DRIVE_SPEED, -40, 40, 40, -40, 8);
                    DRIVE_SPEED = 0.25;
                    encoderDrive(DRIVE_SPEED, -2, -2, -2, -2, 6);
                    DRIVE_SPEED = 0.05;
                    encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 6);
                    DRIVE_SPEED = 0.25;
                    sleep(2500);
                    robot.Warm.setPower(0);
                    encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 8);
                    encoderDrive(DRIVE_SPEED, -10, 10, 10, -10, 8);

                    break;
                }
                else {

                    telemetry.addData("found position", "Center");

                    encoderDrive(DRIVE_SPEED, 3, -3, -3, 3, 4);
                    encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 4);
                    moveSarm(-90, 4);
                    moveBarm(-225, 4);
                    encoderDrive(DRIVE_SPEED, -7, -7, 7, 7, 6);
                    encoderDrive(DRIVE_SPEED, -8.5, -8.5, -8.5, -8.5, 6);
                    robot.Carm.setPosition(0.3); // opens
                    sleep(500);
                    robot.Carm.setPosition(0.53);
                    encoderDrive(DRIVE_SPEED, 7, 7, -7, -7, 7);
                    encoderDrive(DRIVE_SPEED, 7, 7, 7, 7, 7);
                    moveBarm(225, 4);
                    robot.Warm.setPower(0.75);
                    encoderDrive(DRIVE_SPEED, -25, 25, 25, -25, 8);
                    DRIVE_SPEED = 0.1;
                    encoderDrive(DRIVE_SPEED, -7, 7, 7, -7, 4);
                    DRIVE_SPEED = 0.25;
                    sleep(2500);
                    robot.Warm.setPower(0);
                    encoderDrive(DRIVE_SPEED, -8, -8, -8, -8    , 8);
                    encoderDrive(DRIVE_SPEED, -12, 12, 12, -12, 8);

                    break;
                }
            }

            telemetry.update();
        }
    }
    public void testing(ContourPipeline pipeline){
        if(lowerRuntime + 0.05 < getRuntime()){
            crThreshLow += -gamepad1.left_stick_y;
            cbThreshLow += gamepad1.left_stick_x;
            lowerRuntime = getRuntime();
        }
        if(upperRuntime + 0.05 < getRuntime()){
            crThreshHigh += -gamepad1.right_stick_y;
            cbThreshHigh += gamepad1.right_stick_x;
            upperRuntime = getRuntime();
        }

        crThreshLow = inValues(crThreshLow, 0, 255);
        crThreshHigh = inValues(crThreshHigh, 0, 255);
        cbThreshLow = inValues(cbThreshLow, 0, 255);
        cbThreshHigh = inValues(cbThreshHigh, 0, 255);

        pipeline.configureScalarLower(0.0, crThreshLow, cbThreshLow);
        pipeline.configureScalarUpper(255.0, crThreshHigh, cbThreshHigh);

        telemetry.addData("lowerCr ", crThreshLow);
        telemetry.addData("lowerCb ", cbThreshLow);
        telemetry.addData("UpperCr ", crThreshHigh);
        telemetry.addData("UpperCb ", cbThreshHigh);
    }
    public double inValues(double value, double min, double max){
        if(value < min){ value = min; }
        if(value > max){ value = max; }
        return value;
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