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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareRovRuk_1
{
    /* Public OpMode members */
    public DcMotor FrontLeftDrive  = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor RearLeftDrive   = null;
    public DcMotor RearRightDrive  = null;
    //Big Arm
    public DcMotor Barm = null;
    //Small Arm
    public DcMotor Sarm = null;
    //Intake Wheel
    public DcMotor Parm = null;
    //Claw Arm
    public Servo   Carm = null;
    //Wheel Arm
    public CRServo Warm = null;
    public CRServo ArmExtend = null;
    public DigitalChannel digitalTouch;  // Hardware Device Object
    public TouchSensor armLimit; //Magnetic sensor extension limit for the arm
    //public Servo    ServoB    = null;

    public static final double MID_SERVO  =  0.475 ;
    //public DcMotor fLeftEncoder = null;
    //public DcMotor fRightEncoder = null;
    //public DcMotor rLeftEncoder = null;
    //public DcMotor rRightEncoder = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRovRuk_1(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        Barm = hwMap.get(DcMotor.class, "big_arm");
        Sarm = hwMap.get(DcMotor.class, "small_arm");
        Parm = hwMap.get(DcMotor.class, "chicken_parm");
        FrontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        FrontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        RearLeftDrive   = hwMap.get(DcMotor.class, "rear_left_drive");
        RearRightDrive  = hwMap.get(DcMotor.class, "rear_right_drive");
        // get a reference to our digitalTouch object.
        digitalTouch = hwMap.get(DigitalChannel.class, "Finger");
        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        Barm.setDirection (DcMotor.Direction.FORWARD);
        Sarm.setDirection (DcMotor.Direction.FORWARD);
        Parm.setDirection (DcMotor.Direction.FORWARD);
        FrontLeftDrive.setDirection (DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        RearLeftDrive.setDirection  (DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RearRightDrive.setDirection (DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motor

        // Set all motors to zero power and the servos to their init position
        Barm.setPower(0);
        Sarm.setPower(0);
        Parm.setPower(0);
        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        Barm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Sarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Parm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //This is just to separate stuff and make it neat

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //fLeftEncoder = FrontLeftDrive;
        //fRightEncoder = FrontRightDrive;
        //rLeftEncoder = RearLeftDrive;
        //rRightEncoder = RearRightDrive;

        // Define and initialize ALL installed servos.
        Carm  = hwMap.get(Servo.class, "claw_arm");
        Warm  = hwMap.get(CRServo.class, "wheel_arm");
        ArmExtend  = hwMap.get(CRServo.class, "arm_extender");
        Carm.setPosition(MID_SERVO);

        Barm.setPower(0.4);
        while (digitalTouch.getState() == true){

        }
        Barm.setPower(0);


    }
}
