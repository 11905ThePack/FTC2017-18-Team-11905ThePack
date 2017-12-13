package org.firstinspires.ftc.teamcode;

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

/**
 * Created by Christian on 12/4/2017.
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoOp",group="TestAutoOp")

public class AutoOp1 extends LinearOpMode {

//Here Goes to Explaining Varaibles!
// eTime is an abbreviated way of saying Elapsed Time, which says says ow long the robot to drive!
//The DriveMotors are used for actually controlling the robot

    private DcMotor DriveMotorLeft = null;
    private DcMotor DriveMotorRight = null;

    @Override

    public void runOpMode() {

        DriveMotorLeft = hardwareMap.get(DcMotor.class, "DriveMotorLeft");
        DriveMotorRight = hardwareMap.get(DcMotor.class, "DriveMotorRight");

        DriveMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        DriveMotorLeft.setPower(1.0);
        DriveMotorRight.setPower(1.0);

        ElapsedTime eTime = new ElapsedTime();

        eTime.reset();

        while(eTime.time() <2.5){}

        DriveMotorLeft.setPower(0.0);
        DriveMotorRight.setPower(0.0);

        eTime.reset();

        while(eTime.time() <1){}

        DriveMotorLeft.setPower(1.0);

        eTime.reset();

        while(eTime.time() <1){}


        DriveMotorRight.setPower(1.0);

        eTime.reset();

        while(eTime.time() <1);

        DriveMotorLeft.setPower(0.0);
        DriveMotorRight.setPower(0.0);

    }
}