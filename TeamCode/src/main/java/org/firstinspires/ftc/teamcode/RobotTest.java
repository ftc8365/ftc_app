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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot Test", group="Test")
//@Disabled
public class RobotTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private DigitalChannel touchsensor1;  // Hardware Device Object

    boolean isButtonPressed = false;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        servo1  = hardwareMap.get(Servo.class, "servo1");
        servo2  = hardwareMap.get(Servo.class, "servo2");


        // get a reference to our digitalTouch object.
        touchsensor1 = hardwareMap.get(DigitalChannel.class, "touchsensor1");

        // set the digital channel to input.
        touchsensor1.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor1.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double motor1Power = gamepad1.left_stick_y;
            double motor2Power = gamepad1.right_stick_y;
            if (touchsensor1.getState() == false) {
                isButtonPressed = true;

            }

            if ( isButtonPressed == true) {
                motor1Power = 0;

            }
            else {
                motor2Power = 0;
            }

            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);

            if(gamepad1.y) {
                servo1.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {

                servo1.setPosition(0.5);

            } else if (gamepad1.a) {
                servo1.setPosition(1);
            }
            if(gamepad1.y) {
                servo2.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {

                servo2.setPosition(0.5);

            } else if (gamepad1.a) {
                servo2.setPosition(1);
            }


            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (touchsensor1.getState() == true) {
                telemetry.addData("Digital Touch", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch", "Is Pressed");
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "#1 (%.2f)", motor1Power);
            telemetry.update();
        }
    }
}
