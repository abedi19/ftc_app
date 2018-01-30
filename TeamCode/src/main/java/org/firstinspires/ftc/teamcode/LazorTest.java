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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Harold: Lazor Test", group="Harold")

public class LazorTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareHarold robot           = new HardwareHarold();              // Use Harold's hardware
    double          whackerPosition     = 0.0;                   // Servo safe position
    double leftArmPosition= 0.5;
    double rightArmPosition = 0.5;
    //    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
//    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
//    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo
    private ElapsedTime runTime = new ElapsedTime();
    double storedTime = runTime.seconds();


    @Override
    public void runOpMode() {
        double left;
        double right;
        double up;
        double down;
        double position;
        //Laser tag variables:
        double hitNumber = 0;
        double receiverValue;
        double threshHold = 1; //threshHold voltage for recording a hit
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //receiverValue = robot.receiver.getVoltage();
        //digital port := output
        robot.laser.setMode(DigitalChannel.Mode.OUTPUT);
        robot.laser.setState(false); //off

        // set receiver chanel to input.
        robot.receiver.setMode(DigitalChannel.Mode.INPUT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Aman");    //
        telemetry.addData("recievers state is on",robot.laser.getState());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.laser.setState(true);//on

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("recievers state is on",robot.laser.getState());
            telemetry.update();
            position =-1;
            // Run wheels in tank mode
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            if(gamepad1.right_bumper){
                robot.leftMotor.setPower(0.4*left);
                robot.rightMotor.setPower(0.4*right);
            }  else {
                robot.leftMotor.setPower(left);
                robot.rightMotor.setPower(right);
            }


            // triggers lifters
            if(gamepad1.left_trigger >0) {
                robot.lifter.setPower(1*gamepad1.left_trigger);
            } else if (gamepad1.right_trigger>0) {
                robot.lifter.setPower(-1*gamepad1.right_trigger);
            } else {
                robot.lifter.setPower(0.0);

            }

            //TODO:Preset positions
            if(gamepad1.dpad_down) {
                robot.lifter.setTargetPosition(-600);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1.0);
                while (opModeIsActive() && robot.lifter.isBusy()) {
                    // wait
                    // added driving
                    left = gamepad1.left_stick_y;
                    right = gamepad1.right_stick_y;
                    if(gamepad1.right_bumper){
                        robot.leftMotor.setPower(0.4*left);
                        robot.rightMotor.setPower(0.4*right);
                    }  else {
                        robot.leftMotor.setPower(left);
                        robot.rightMotor.setPower(right);
                    }
                }
                // Stop all motion;
                robot.lifter.setPower(0);
                robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.dpad_left) {
                robot.lifter.setTargetPosition(-3275);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1.0);
                while (opModeIsActive() && robot.lifter.isBusy()) {
                    // wait boyo
                    left = gamepad1.left_stick_y;
                    right = gamepad1.right_stick_y;
                    if(gamepad1.right_bumper){
                        robot.leftMotor.setPower(0.4*left);
                        robot.rightMotor.setPower(0.4*right);
                    }  else {
                        robot.leftMotor.setPower(left);
                        robot.rightMotor.setPower(right);
                    }
                }
                // Stop all motion;
                robot.lifter.setPower(0);
                robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.dpad_right) {
                robot.lifter.setTargetPosition(-5752);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1.0);
                while (opModeIsActive() && robot.lifter.isBusy()) {
                    // wait boyo
                    left = gamepad1.left_stick_y;
                    right = gamepad1.right_stick_y;
                    if(gamepad1.right_bumper){
                        robot.leftMotor.setPower(0.4*left);
                        robot.rightMotor.setPower(0.4*right);
                    }  else {
                        robot.leftMotor.setPower(left);
                        robot.rightMotor.setPower(right);
                    }
                }
                // Stop all motion;
                robot.lifter.setPower(0);
                robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if(gamepad1.dpad_up) {
                robot.lifter.setTargetPosition(-1830);
                robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lifter.setPower(1.0);
                while (opModeIsActive() && robot.lifter.isBusy()) {
                    // wait boyo
                    left = gamepad1.left_stick_y;
                    right = gamepad1.right_stick_y;
                    if(gamepad1.right_bumper){
                        robot.leftMotor.setPower(0.4*left);
                        robot.rightMotor.setPower(0.4*right);
                    }  else {
                        robot.leftMotor.setPower(left);
                        robot.rightMotor.setPower(right);
                    }
                }
                // Stop all motion;
                robot.lifter.setPower(0);
                robot.lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


//
            //TODO: Servos
            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a) {
                whackerPosition = 0;
            }
            else if (gamepad1.y) {
                whackerPosition = 0.4;
            }
            if(gamepad1.x){
                leftArmPosition -= 0.1;
                rightArmPosition +=0.1;
            } else if (gamepad1.b){
                leftArmPosition += 0.1;
                rightArmPosition -=0.1;
            }
            if(gamepad1.left_bumper){
                position = robot.lifter.getCurrentPosition();
            }

//left is acutally right and right ascutally left
//            // Move both servos to new position.
            // at 0 left is all the way in and right is all the way out
            //   whackerPosition  = Range.clip(whackerPosition, 0.0, 0.6);
            leftArmPosition = Range.clip(leftArmPosition, 0.15, 0.7); // GM: Changed min parameter to accommodate new servo mount position
            rightArmPosition = Range.clip(rightArmPosition, 0.15,0.9);
            robot.whacker.setPosition(whackerPosition);
            robot.leftArm.setPosition(leftArmPosition);
            robot.rightArm.setPosition(rightArmPosition);

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);


//TODO: LAZOR STUFF
            // Analog Receiver:
//            receiverValue = robot.receiver.getVoltage();
//            if(receiverValue > threshHold){
//                hitNumber ++;
//                telemetry.addData("hitNumber","I'm hit");
//                telemetry.addData("hitNumber",hitNumber);
//                telemetry.update();
//                sleep(5000);
//            }

            //Laser:
//            if(gamepad1.left_bumper){
//                 storedTime = runTime.seconds();
//                while(runTime.seconds()-storedTime < 1.5) {
//                    robot.laser.setState(true);
//                }
//                robot.laser.setState(false);
//            }



            telemetry.update();
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
        robot.lifter.setPower(0.0);
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);
    }
}
