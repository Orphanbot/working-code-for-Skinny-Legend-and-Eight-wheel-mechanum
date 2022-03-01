/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.index.qual.Positive;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Eight Wheel Drive", group="Linear Opmode")
//@Disabled
public class Eight_Wheel_Drive extends LinearOpMode {

    Hardware_EWD robot           = new Hardware_EWD();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    // DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {

        double POWER = 0.7;
        double front_left;
        double front_right;
        double rear_left;
        double rear_right;
        double forward; // push joystick1 forward to go forward
        double strafe; // push joystick1 to the right to strafe right
        double clockwise; // push joystick2 to the right to rotate clockwise

        // get a reference to our digitalTouch object.
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            forward = -gamepad1.left_stick_y;
            strafe =  gamepad1.left_stick_x;
            clockwise = gamepad1.right_stick_x;

            front_left = forward + clockwise + strafe;
            front_right = forward - clockwise - strafe;
            rear_left = forward + clockwise - strafe;
            rear_right = forward - clockwise + strafe;

            front_left    = Range.clip(front_left, -1, 1) ;
            front_right   = Range.clip(front_right, -1, 1) ;
            rear_left    = Range.clip(rear_left, -1, 1) ;
            rear_right   = Range.clip(rear_right, -1, 1) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.FrontLeftDrive.setPower(front_left * POWER);
            robot.FrontRightDrive.setPower(front_right * POWER);
            robot.RearLeftDrive.setPower(rear_left * POWER);
            robot.RearRightDrive.setPower(rear_right * POWER);

            if (gamepad1.right_bumper) {
                POWER = 1.0;
            } else {
                POWER = 1.0;
            }

            if ((gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3) &&
                    (gamepad1.right_stick_x < 0.3 && gamepad1.right_stick_x > -0.3) &&
                    (gamepad1.left_stick_y  < 0.3 && gamepad1.left_stick_y > -0.3)){
                POWER = 0;
            }

            // Hey, connor, lucas here, I tampered here.
            if (gamepad1.dpad_up) {
                robot.LBarm.setPower(-1);
                robot.RBarm.setPower(1);
            } else if (gamepad1.dpad_down) {
                robot.LBarm.setPower(1);
                robot.RBarm.setPower(-1);
            } else
                robot.LBarm.setPower(0);
                robot.RBarm.setPower(0);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front_left: " + front_left);
            telemetry.addData("Motors", "front_right: " + front_right);
            telemetry.addData("Motors", "rear_left: " + rear_left);
            telemetry.addData("Motors", "rear_right: " + rear_right);
            telemetry.update();
        }
    }
}
