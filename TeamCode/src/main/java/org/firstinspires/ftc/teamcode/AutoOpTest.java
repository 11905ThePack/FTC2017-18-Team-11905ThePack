package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name ="TestAutoOp")

public class AutoOpTest extends LinearOpMode {

    private DcMotor DriveLeftRear = null;
    private DcMotor DriveLeftFront = null;
    private DcMotor DriveRightRear = null;
    private DcMotor DriveRightFront = null;

    ElapsedTime eTime = new ElapsedTime();

    public void runOpMode() {

        DriveLeftRear = hardwareMap.get(DcMotor.class, "DriveLeftRear");
        DriveLeftFront = hardwareMap.get(DcMotor.class, "DriveLeftFront");
        DriveRightRear = hardwareMap.get(DcMotor.class,"DriveRightRear");
        DriveRightFront = hardwareMap.get(DcMotor.class, "DriveRightFront");

        DriveLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }


}
