package org.firstinspires.ftc.teamcode;

/*
 *This file defines a Java class that performs the configuration and setup for our robot's
 *  hardware (motors and sensors).
 * It assumes 5 motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive, and arm) and
 * 2 servos (left_hand and right_hand)
 *
 * This one class can be used by ALL of our OpMode without having to cut and paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls
 * into the class, rather than accessing the internet hardware directly. This is why the objects are declared "private".
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {
    // Declare OpModes members.
    private LinearOpMode myOpMode = null;    // gain access to the methods in the calling OpMode.

    // Define Motor and Servo objects (Make them private so they can't ve accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    //Define drive constants. Make them public so they can be used by the ceiling OpMode
    public static final double MID_SERVO      = 0.5;
    public static final double HAND_SPEED     = 0.02;
    public static final double ARM_UP_POWER   = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware (LinearOpMode opmode)  {myOpMode = opmode;}

    /**
     * Initialise all the robot;s hardware. This method must be called ONCE when the OpMode is Initialised.
     * </p>
     * All the hardware devices are accessed via the hardware map, and Initialised.
     */

    public void init() {
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        //arm   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        /* To drive forward, our robot need the motors on one side to be reversed, because the axles point in opposite
        direction
        Pushing the left stick forward MUST make robot go forward. So we will adjust these line if necessary, after
        the first test drive,
         */

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Define and Initialize ALL Installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     *
     *  Calculates the left/right motor powers required to achieve the requested
     *  robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   Left?Right driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    public void driveRobot(double axial, double lateral, double yaw) {
        double max;

        // Combine drive and turn for blended motion.
        double leftFrontPower  = axial + lateral + yaw;
        double leftBackPower = axial - lateral - yaw;
        double rightFrontPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;


        //Scale the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(leftFrontPower) ,  Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0)  {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower (leftFrontPower, rightFrontPower, leftBackPower, rightBackPower );

    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontWheel     Fed/Rev driving power {-1.0 to 1.0}  +ve is forward
     * @param leftBackWheel
     * @param rightFrontWheel
     * @param rightBackWheel    Fed/Rev driving power (-1.0 to 1.0)  +ve is forward
     */

    public void setDrivePower(double leftFrontWheel, double leftBackWheel,  double rightFrontWheel, double rightBackWheel) {
        leftFrontDrive.setPower(leftFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightFrontDrive.setPower((rightFrontWheel));
        rightBackDrive.setPower(rightBackWheel);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */

    public void setArmPower(double power) {arm.setPower(power); }

    /**
     * send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */

    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);

    }

}
