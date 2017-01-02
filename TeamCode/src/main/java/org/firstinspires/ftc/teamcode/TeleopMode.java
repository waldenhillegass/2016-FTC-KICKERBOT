/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;


import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Template: Iterative OpMode", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice
@Disabled
public class TeleopMode extends OpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private double leftThrottle = 0;
    private double rightThrottle = 0;
    private final double LIGHT_THRESHOLD = 100;
    private boolean isFirstTime = true;
     private int targetHeading = 0;
    private boolean flippersLocked = true;

    HocoHardware robot = new HocoHardware();






    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        robot.init(hardwareMap);
        robot.gyro.calibrate();


        if (getBatteryVoltage() < 12) telemetry.addData("BATTERY VOLTAGE LOW", getBatteryVoltage());






        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (!robot.gyro.isCalibrating()) {
            telemetry.addData("gryo calibrated!", robot.gyro.getHeading());

        } else {
            telemetry.addData("gryo is calibrating", "");
        }
        telemetry.update();
    }


    @Override
    public void start() {
        //telemetry.addData("Blast Off", " All Systems are GO!");
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        leftThrottle = -gamepad1.left_stick_y;
        rightThrottle = -gamepad1.right_stick_y;


            //makes the throttle less sensitive if right bumper is pressed.

        if (gamepad1.left_bumper){
            if (robot.groundSensor.alpha() > LIGHT_THRESHOLD) {
                leftThrottle = 0;
                leftThrottle = 0;
            }
        }


        if (gamepad2.a) {
            flippersLocked = true;
        }
        if (gamepad2.left_trigger != 0 || gamepad2.right_trigger != 0) {
            flippersLocked = false;
        }


        if(gamepad1.x){
            robot.groundSensor.enableLed(true);
        }
        if(gamepad1.y){
            robot.groundSensor.enableLed(false);
        }


        if (gamepad2.dpad_down) robot.foot.setPosition(-1);
        if (gamepad2.dpad_up) robot.foot.setPosition(.9);

        if (!flippersLocked) {
            robot.leftie.setPosition(gamepad2.left_trigger);
            robot.rightie.setPosition(-gamepad2.right_trigger + 1);
        } else {
            robot.leftie.setPosition(1);
            robot.rightie.setPosition(0);
        }


        if (gamepad2.right_bumper) {
            // if(Math.abs(robot.rack.getCurrentPosition())<6000){
            robot.rack.setPower(1);
            // }
            // if(gamepad2.y){
            //   robot.rack.setPower(1);
            // }
        } else if (gamepad2.left_bumper)
            robot.rack.setPower(-1);
        else {
            robot.rack.setPower(0);
        }
        telemetry.addData("Rack Encoder:", robot.rack.getCurrentPosition());

//elchawpawashere
        //flexonurxbox
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);
        leftThrottle = clipRanges(leftThrottle);
        rightThrottle = clipRanges(rightThrottle);


        robot.rightMotor.setPower(rightThrottle);
        robot.leftMotor.setPower(leftThrottle);

        if (!(gamepad2.left_stick_y == 0)) {
            robot.foot.setPosition(gamepad2.left_stick_y);
            telemetry.addData("", robot.foot.getPosition());
        }

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Nice Job Driver!", "");
        telemetry.update();
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }



    double clipRanges(double whatIAmClipping){

        whatIAmClipping = Math.min(whatIAmClipping, 1);
        whatIAmClipping = Math.max (whatIAmClipping , -1);

        return whatIAmClipping;

    }
}


