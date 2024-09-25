package org.firstinspires.ftc.teamcode;

/*
*this OpMode Illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
* This approach is very efficient because the same hardware class can be used by all of your teleOp and autonomous OpModes
* without requiring many copy & paste operations. Once you have defined and tested the hardware class with one OpMode,
* It is instantly available to other OpMOdes.
*
* The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place
* (the Hardware class). So, to be effective you should put as much or your hardware setup and access code as possible in
* the hardware class.
*
* The Hardware class is created in a separate file, and then an "instance" of this class is created in each OpMode.
* In order for the class to do typical OpMode things (like send telemetry data) It must be passed a reference to the
* OpMode object when it's created, so it can access all core OpMode functions.
*
*
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name= "Concept: Robot Hardware Class", group= "Robot")

public class RobotCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    //Prefix any hardware functions with "robot." to access this class.

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMOde() {
        double axial       = 0;
        double lateral     = 0;
        double yaw         = 0;
        double arm         = 0;
        double handOffset  = 0;

        //initialize all the hardware using the hardware class. See this? Very simple. Very clean. Very demure.
        robot.init();

        //Send telemetry message to simplify robot waiting;
        //Wait for the game to start (driver presses PLAY)
        waitForStart();

        //run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            * Run wheels in POV mode (not: The joystick goes negative when pushed forward, so we'll negate it)
            * In this mode the left stick moves the robot fwd and back, the Right stick turns left and right.
            * This way it's also easy to just drive straight, or just turn.
             */
            axial= -gamepad1.left_stick_y;
            lateral= gamepad1.left_stick_x;
            yaw= gamepad1.right_stick_x;

            //combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw);

            /*
            * Use gamepad left and right Bumpers to open and close the claw
            * Use the SERVO constants defined in RobotHardware class.
            * Each time around the loop, the servos will move by a small amount.
            * Limit the total offset to half of the full travel range
            *
             */

            if (gamepad1.right_bumper) {
                handOffset += robot.HAND_SPEED;
            }
            else if (gamepad1.left_bumper) {
                handOffset -= robot.HAND_SPEED;
            }
            handOffset = Range.clip(handOffset, -0.5, 0.5);

            //Move both servos to new position. Use RobotHardware class
            robot.setHandPositions(handOffset);

            // Use gamepad buttons to move arm up and down (y and a respectively)
            //Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y){
                arm = robot.ARM_UP_POWER;
            }
            else if (gamepad1.a) {
                arm = robot.ARM_DOWN_POWER;
            }
            else{
                arm = 0;
            }

            robot.setArmPower(arm);

            //Send telemetry messages to explain controls and show robot status
            telemetry.addData("Axial", "left_stick_y");   // Axial means driving up and down
            telemetry.addData("Lateral", "left_stick_x");     // lateral means driving side to side
            telemetry.addData("Yaw", "right_stick_x"); // Don't be afraid. Yaw means Turning
            telemetry.addData("Arm Up/Down", "Y & A buttons");
            telemetry.addData("Hand Open/Closed", "left and right bumpers");
            telemetry.addData("-", "--------");

             telemetry.addData("Axial Power", "%2f", axial);
             telemetry.addData("Lateral Power ", "%2f", lateral);
             telemetry.addData("Yaw Power", "%2f", yaw);
             telemetry.addData("Arm Power", "%2f", arm);
             telemetry.addData("Hand Position", "Offset = %2f", handOffset);
             telemetry.update();

             // We'll pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
