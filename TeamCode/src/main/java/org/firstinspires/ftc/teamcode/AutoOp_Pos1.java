package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


// Redistribution and use in source and binary forms, with or without modification,
// are permitted (subject to the limitations in the disclaimer below) provided that
// the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list
// of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
//
// Neither the name of FIRST nor the names of its contributors may be used to endorse or
// promote products derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//Created by Christian on 12/4/2017.

@Autonomous(name="AutoOp1",group="TestAutoOp")

public class AutoOp_Pos1 extends LinearOpMode {

//Here Goes to Explaining Variables!
// eTime is an abbreviated way of saying Elapsed Time, which says says ow long the robot to drive!
//The DriveMotors are used for actually controlling the robotoo


    private DcMotor DriveLeftRear = null;
    private DcMotor DriveLeftFront = null;
    private DcMotor DriveRightRear = null;
    private DcMotor DriveRightFront = null;

    private Servo GlyphServoLeft = null;
    private Servo GlyphServoRight = null;

    private DcMotor MotorGlyphGrabber = null;

    ElapsedTime eTime = new ElapsedTime();

    private double servoGlyphLeftPosition = 1; //180 degrees
    private double servoGlyphRightPosition = 0;



    @Override

    public void runOpMode() {

        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightRear = hardwareMap.get(DcMotor.class,"DriveRightRear");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");
        MotorGlyphGrabber = hardwareMap.get(DcMotor.class," MotorGlyphGrabber");

        DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        GlyphServoLeft = hardwareMap.get(Servo.class, "GlyphServoLeft");
        GlyphServoRight = hardwareMap.get(Servo.class, "GlyphServoRight");

        waitForStart();

        DriveLeftRear.setPower(0);
        DriveLeftFront.setPower(0);
        DriveRightRear.setPower(0);
        DriveRightFront.setPower(0);

        eTime.reset();

        while (eTime.time()<1.0) {}

        GlyphServoLeft.setPosition(80/180);
        GlyphServoRight.setPosition(100/180);
        MotorGlyphGrabber.setPower(1);
        //LIFT GlYPH HERE


        eTime.reset();

        while (eTime.time()<1.3) {}
//Drive in Reverse
        DriveLeftRear.setPower(-0.15);
        DriveLeftFront.setPower(-0.15);
        DriveRightRear.setPower(0.15);
        DriveRightFront.setPower(0.15);
        MotorGlyphGrabber.setPower(0);
        eTime.reset();

        while (eTime.time()<6) {}
//Turn 180 degrees
        DriveLeftRear.setPower(0.15);
        DriveLeftFront.setPower(0.15);
        DriveRightRear.setPower(0.15);
        DriveRightFront.setPower(0.15);

        eTime.reset();
//Add drive forward  little bit to put the glyph in the crypto box.
        while (eTime.time()<0.2) {}

        DriveLeftRear.setPower(0.15);
        DriveLeftFront.setPower(0.15);
        DriveRightRear.setPower(-0.15);
        DriveRightFront.setPower(-0.15);

        eTime.reset();

        while (eTime.time()<1) {}
        //Drop Glyph
        GlyphServoLeft.setPosition(1);
        GlyphServoRight.setPosition(0);

        eTime.reset();

        while (eTime.time()<0.2) {} //Drive Forward --this line is NOT driving forward!

        //Push glyph further in

        DriveLeftRear.setPower(0.15);
        DriveLeftFront.setPower(0.15);
        DriveRightRear.setPower(-0.15);
        DriveRightFront.setPower(-0.15);


        eTime.reset();

        while (eTime.time()<0.3) {} //Drive Backward

        DriveLeftRear.setPower(-0.15);
        DriveLeftFront.setPower(-0.15);
        DriveRightRear.setPower(0.15);
        DriveRightFront.setPower(0.15);

        eTime.reset();


        //resets motors
        while (eTime.time()<1) {}

        DriveLeftRear.setPower(0);
        DriveLeftFront.setPower(0);
        DriveRightRear.setPower(0);
        DriveRightFront.setPower(0);

    }
}
