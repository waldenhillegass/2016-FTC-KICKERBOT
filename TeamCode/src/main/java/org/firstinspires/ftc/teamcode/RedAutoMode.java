package org.firstinspires.ftc.teamcode;/*
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoRedTeam", group="Autonomous")  // @Autonomous(...) is the other common choice
@Disabled
public class RedAutoMode extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private int autoState = 1;
    private double leftThrottle, rightThrottle;
    ProgbotHardware robot = new ProgbotHardware();
    private final double LIGHT_THRESHOLD = 4;
    //private boolean isFirstTime = true;
    //private int targetHeading = 0;
    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);
        robot.gyro.calibrate();
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
        if (!robot.gyro.isCalibrating()){
            telemetry.addData("gryo calibrated!", robot.gyro.getHeading());

        }else{
            telemetry.addData("gryo is calibrating", "");
        }
        telemetry.addData("Ultrasonic", robot.getUltrasonicDistance());

        telemetry.update();

        if (getRuntime() > 15000){
            robot.gyro.calibrate();
        }

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        switch (autoState){
            case 1:
                //the target heading is 315 from the heading zero that the robot should be positioned at now.
                driveToHeading(42, -.5, 5);
                if (Math.abs(robot.gyro.getHeading() - 40) < 5)
                {
                    autoState++;
                    resetDriveEncoders();
                }

                break;

            case 2: //This Case drives the robot straight for 12.56 rotations

                //there are 1440 ticks per each revolution
                rightThrottle = (.5);
                leftThrottle = (.5);
                if (averageEncoders() > inchesToTicks(4, 37)) {
                    resetDriveEncoders();
                    autoState++;
                    leftThrottle = 0;
                    rightThrottle = 0;
                }
                break;
            //Target heading is zero

            case 3:
                driveToHeading(3, -.5, 5);
                if (robot.gyro.getHeading() < 3) {
                    autoState++;
                    resetDriveEncoders();
                }
                break;
            case 4:
                rightThrottle = .1;
                leftThrottle = .1;
                if (averageEncoders() > 2000 || robot.groundSensor.red() > LIGHT_THRESHOLD) {
                    autoState++;
                    resetDriveEncoders();
                    rightThrottle = 0;
                    leftThrottle = 0;

                }
                break;
            case 5:
                rightThrottle = 0;
                leftThrottle = 0;
            default:
                telemetry.addData("An error has occured", "");
                break;






        }
        telemetry.addData("Current Heading", robot.gyro.getHeading());
        telemetry.addData("AutoState", autoState);
        telemetry.addData("Average Encoders", averageEncoders());
        telemetry.addData("Left", leftThrottle);
        telemetry.addData("Right", rightThrottle);
        telemetry.addData("Ground Color Red", robot.groundSensor.red());
        robot.rightMotor.setPower(rightThrottle);
        robot.leftMotor.setPower(leftThrottle);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    void resetDriveEncoders(){
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    int averageEncoders(){
        return (robot.leftMotor.getCurrentPosition() + robot.rightMotor.getCurrentPosition()) / 2;
    }

    private void driveToHeading(double target, double speed, double correctionfactor) {
        double current = robot.gyro.getHeading();
        double difference = target - current;
        while (difference > 180) {
            difference = difference - 360;
        }

        while (difference < -180) {
            difference = difference + 360;
        }
        double correction = difference / correctionfactor;

        double rightMotor = speed + (correction * speed);
        double leftMotor = speed - (correction * speed);

        rightMotor = Math.max(0, Math.min(1, rightMotor));
        leftMotor = Math.max(0, Math.min(1, leftMotor));

        rightThrottle = rightMotor;
        leftThrottle = leftMotor;


    }

    public double inchesToTicks(int diameter, double dist) {
        double circ = (diameter) * 3.14;
        double answer = dist / circ;
        answer = answer * 1440;
        return answer;
    }


}