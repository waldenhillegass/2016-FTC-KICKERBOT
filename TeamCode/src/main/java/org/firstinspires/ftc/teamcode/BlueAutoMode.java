package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by walden on 1/2/17.
 */
@Autonomous(name = "AutoBlueTeam", group = "Autonomous")
// @Autonomous(...) is the other common choice
@Disabled
public class BlueAutoMode extends OpMode {


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
        if (!robot.gyro.isCalibrating()) {
            telemetry.addData("gryo calibrated!", robot.gyro.getHeading());

        } else {
            telemetry.addData("gryo is calibrating", "");
        }
        telemetry.addData("Ultrasonic", robot.getUltrasonicDistance());

        telemetry.update();

        if (getRuntime() > 15000) {
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

        switch (autoState) {
            case 1:
                //the target heading is 315 from the heading zero that the robot should be positioned at now.
                driveToHeading(318, -.5, 5);
                if (Math.abs(robot.gyro.getHeading() + 40) > 355) {
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


    void resetDriveEncoders() {
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    int averageEncoders() {
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
