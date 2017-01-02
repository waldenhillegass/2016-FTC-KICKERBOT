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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 * <p/>
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 * <p/>
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 * <p/>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "K9bot: Telop Tank", group = "K9bot")
@Disabled
public class DriveToHeading extends OpMode {

    /* Declare OpMode members. */
    ProgbotHardware robot = new ProgbotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    // Use a K9'shardware
    private double leftThrottle = 0;
    private double rightThrottle = 0;
    double correctionFactor = 1;
    double targetHeading = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.gyro.calibrate();
        rightThrottle = 0;
        leftThrottle = 0;

    }

    @Override
    public void init_loop() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        if (robot.gyro.isCalibrating()) {
            telemetry.addData("Gyro is calibrating. Please do not move robot.", "");
        } else {
            telemetry.addData("Done.", "");
        }
        telemetry.update();


        // Send telemetry message to signify robot waiting;
        //   telemetry.addData("Say", "Dick");

        // Wait for the game to start (driver presses PLAY)


        // run until the end of the match (driver presses STOP)
    }

    @Override
    public void start() {
        runtime.reset();
        rightThrottle = 0;
        leftThrottle = 0;
    }

    @Override
    public void loop() {

        if (correctionFactor <= 0) correctionFactor = 1;

        driveToHeading(targetHeading, gamepad1.right_trigger, correctionFactor);

        if (gamepad1.left_bumper) {
            leftThrottle = -1;
            rightThrottle = -1;
        }

        if (gamepad1.right_bumper) {
            leftThrottle = 1;
            rightThrottle = 1;
        }

        if (gamepad1.dpad_right) {
            if (targetHeading > 360) {
                targetHeading = 0;
            } else {
                targetHeading += .5;
            }

        } else if (gamepad1.dpad_left) {
            if (targetHeading < 0) {
                targetHeading = 359;
            } else {
                targetHeading -= .25;
            }
        }

        if (gamepad1.dpad_up) {
            correctionFactor += .25;
        } else if (gamepad1.dpad_down) {
            correctionFactor -= .25;
        }


        telemetry.addData("Left", leftThrottle);
        telemetry.addData("Right", rightThrottle);
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Correction Factor", correctionFactor);
        telemetry.addData("Current Heading", robot.gyro.getHeading());
        telemetry.update();

        leftThrottle = Range.clip(leftThrottle, -1, 1);
        rightThrottle = Range.clip(rightThrottle, -1, 1);


        robot.rightMotor.setPower(rightThrottle);
        robot.leftMotor.setPower(leftThrottle);
//ifsvYmcf
        // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

    }


    public void driveToHeading(double target, double speed, double correctionFactor) {

        double current = robot.gyro.getHeading();
        double difference = target - current;
        while (difference > 180) {
            difference = difference - 360;
        }

        while (difference < -180) {
            difference = difference + 360;
        }
        double correction = difference / correctionFactor;

        double rightMotor = speed + (correction);
        double leftMotor = speed - (correction);

        rightMotor = Math.max(0, Math.min(1, rightMotor));
        leftMotor = Math.max(0, Math.min(1, leftMotor));

        rightThrottle = rightMotor;
        leftThrottle = leftMotor;
    }

    double clipRanges(double whatIAmClipping) {

        whatIAmClipping = Math.min(whatIAmClipping, 1);
        whatIAmClipping = Math.max(whatIAmClipping, -1);

        return whatIAmClipping;

    }
}
