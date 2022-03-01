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

import static java.lang.Math.abs;

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

@TeleOp(name="Skinny Legend drive", group="Linear Opmode")
//@Disabled
public class Mecanum_Drive_1 extends LinearOpMode {

    HardwareRovRuk_1 robot           = new HardwareRovRuk_1();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor leftDrive = null;
    //private DcMotor rightDrive = null;
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;// sets rate to move servo

    // DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void runOpMode() throws InterruptedException {


        double POWER = 1;
        double front_left;
        double front_right;
        double rear_left;
        double rear_right;
        double forward; // push joystick1 forward to go forward
        double strafe; // push joystick1 to the right to strafe right
        double clockwise; // push joystick2 to the right to rotate clockwisEe

        // get a reference to our digitalTouch object.
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        robot.init(hardwareMap);

        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        robot.Carm.setPosition(0.3);

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

            if (gamepad2.dpad_up){
                robot.Barm.setPower(-0.6);
            } else if (gamepad2.dpad_down && robot.digitalTouch.getState() == true){
                robot.Barm.setPower(0.6);
            } else if (gamepad2.dpad_down && robot.digitalTouch.getState() == false){
                robot.Barm.setPower(0);
            } else {
                robot.Barm.setPower(0);
            }

            if (gamepad1.dpad_right)
                robot.Sarm.setPower(-0.5);
            else if (gamepad1.dpad_left)
                robot.Sarm.setPower(0.5);
            else
                robot.Sarm.setPower(0);

            if (gamepad1.a)
                robot.Parm.setPower(0.85);
            else if (gamepad1.b)
                robot.Parm.setPower(-0.85);
            else
                robot.Parm.setPower(0);

            if (gamepad1.right_bumper) {
                POWER = 1;
            } else {
                POWER = 1;
            }

            if ((gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3) &&
                    (gamepad1.right_stick_x < 0.3 && gamepad1.right_stick_x > -0.3) &&
                    (gamepad1.left_stick_y  < 0.3 && gamepad1.left_stick_y > -0.3)){
                POWER = 0;
            }

            //servo controls
           if (gamepad2.a) {
               clawOffset += CLAW_SPEED;
               telemetry.addData("servo close", clawOffset);
           } else if (gamepad2.b) {
               clawOffset -= CLAW_SPEED;
               telemetry.addData("servo open", clawOffset);
           }

            if (gamepad2.x) {
                robot.Warm.setPower(-1);
            } else if (gamepad2.y) {
                robot.Warm.setPower(1);
            } else {
                robot.Warm.setPower(0);
            }

            if (gamepad2.dpad_left)
                robot.ArmExtend.setPower(-1);
            else if (gamepad2.dpad_left && robot.armLimit.isPressed())
                robot.ArmExtend.setPower(0);
            else if (gamepad2.dpad_right)
                robot.ArmExtend.setPower(1);
            else
                robot.ArmExtend.setPower(0);


            if (robot.digitalTouch.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }

            // clawOffset = Range.clip(clawOffset, -0.45, -0.05);
            clawOffset = Range.clip(clawOffset, 0.3, 0.65);

            telemetry.addData("clawoffset after range.clip", clawOffset);
            robot.Carm.setPosition(clawOffset);

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
