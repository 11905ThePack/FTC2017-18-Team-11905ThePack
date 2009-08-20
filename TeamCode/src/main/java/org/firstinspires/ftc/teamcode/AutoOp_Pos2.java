package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Ken on 1/26/2018.
 */
@Autonomous(name="AutoOp_Test2",group="TestAutoOp")
public class AutoOp_Pos2 extends LinearOpMode {

    ColorSensor colorSensor;



    ElapsedTime eTime2 = new ElapsedTime();

    private DcMotor DriveLeftRear = null;
    private DcMotor DriveLeftFront = null;
    private DcMotor DriveRightRear = null;
    private DcMotor DriveRightFront = null;
    private Servo GlyphServoLeft = null;
    private Servo GlyphServoRight = null;

    private DcMotor MotorGlyphGrabber = null;

    ElapsedTime eTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "JewelWhackerColorSensor");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // check the status of the x button on either gamepad.
            while (eTime2.time() < 3)
                bCurrState = true;
            eTime2.reset();


            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState)) {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            if (colorSensor.red() >= 2) {

                telemetry.addData("Bleh", "Red");

                DriveLeftRear.setPower(-0.2);
                DriveLeftFront.setPower(-0.2);
                DriveRightRear.setPower(0.2);
                DriveRightFront.setPower(0.2);

            } else {
                telemetry.addData("Bleh", "blue");

                DriveLeftRear.setPower(0.2);
                DriveLeftFront.setPower(0.2);
                DriveRightRear.setPower(0.2);
                DriveRightFront.setPower(0.2);

            }


            telemetry.update();

        }

        while (opModeIsActive()) {

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

}

