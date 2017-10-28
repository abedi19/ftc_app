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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwareK9bot;

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

@TeleOp(name="Harold: Telop Tank", group="Harold")

public class HaroldTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareHarold robot           = new HardwareHarold();              // Use Harold's hardware
   double          whackerPosition     = 0.0;                   // Servo safe position
    double leftArmPosition= 0.5;
    double rightArmPosition = 0.5;
//    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
//    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
//    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;
        double up;
        double down;
        double position;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Aman");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            position =-1;
            // Run wheels in tank mode
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            if(gamepad1.right_bumper){
                robot.leftMotor.setPower(0.4*left);
                robot.rightMotor.setPower(0.4*right);
            } else {
                robot.leftMotor.setPower(left);
                robot.rightMotor.setPower(right);
            }



            // triggers lifters
            if(gamepad1.left_trigger >0 && (robot.lifter.getCurrentPosition() > -11.2) ) {
                robot.lifter.setPower(1*gamepad1.left_trigger);
            } else if (gamepad1.right_trigger>0 && (robot.lifter.getCurrentPosition() < -12921.3)){
                robot.lifter.setPower(-1*gamepad1.right_trigger);
            } else {
                robot.lifter.setPower(0.0);

            }
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
//
//            // Use gamepad X & B to open and close the claw
//            if (gamepad91.x)
//                clawPosition += CLAW_SPEED;
//            else if (gamepad1.b)
//                clawPosition -= CLAW_SPEED;
//
//            // Move both servos to new position.
            // at 0 right is all the way in and left is all the way out
            whackerPosition  = Range.clip(whackerPosition, 0.0, 0.6);
            rightArmPosition = Range.clip(rightArmPosition, 0.3, 0.8);
            leftArmPosition = Range.clip(leftArmPosition, 0.3,0.8);
            robot.whacker.setPosition(whackerPosition);
            robot.leftArm.setPosition(leftArmPosition);
            robot.rightArm.setPosition(rightArmPosition);
//            robot.arm.setPosition(armPosition);
//            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
//            robot.claw.setPosition(clawPosition);
//
//            // Send telemetry message to signify robot running;
//            telemetry.addData("arm",   "%.2f", armPosition);
//            telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.addData("position","%.2f", position);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
