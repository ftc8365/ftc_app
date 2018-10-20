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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


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

@TeleOp(name="Flywheel Demo", group="Demo")
@Disabled
public class FlywheelDemo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;

    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBR = null;
    private DcMotor motorBL = null;

    private Servo servo1 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor1  = hardwareMap.get(DcMotor.class, "motor1");
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");

        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);

        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);

        motorBR  = hardwareMap.get(DcMotor.class, "motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        motorBL  = hardwareMap.get(DcMotor.class, "motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);



        servo1  = hardwareMap.get(Servo.class, "servo1");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motor1.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.b)
                servo1.setPosition(1);
            else
                servo1.setPosition(0.5);


            // Setup a variable for each drive wheel to save power level for telemetry
            double motorPower = gamepad2.right_stick_y;

            double scoopPower = gamepad2.left_stick_y;

            if (scoopPower < 0)
                scoopPower = scoopPower / 2.5;
            else
                scoopPower = scoopPower / 4;

            motor1.setPower(motorPower);
            motor2.setPower(motorPower);
            motor3.setPower(scoopPower);

            double powerFR = 0;
            double powerFL = 0;
            double powerBR = 0;
            double powerBL = 0;

            double x1 = gamepad1.right_stick_x;
            double y1 = gamepad1.right_stick_y * -1;

            double x2 = gamepad1.left_stick_x;

            powerFR -= x2;
            powerFL += x2;
            powerBR -= x2;
            powerBL += x2;

            powerFR += y1;
            powerFL += y1;
            powerBR += y1;
            powerBL += y1;


            powerFR -= x1;
            powerFL += x1;
            powerBR += x1;
            powerBL -= x1;

            motorFR.setPower(powerFR);
            motorFL.setPower(powerFL);
            motorBR.setPower(powerBR);
            motorBL.setPower(powerBL);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "#1 (%.2f)", motorPower);
            telemetry.addData("Scoop", "#1 (%.2f)", scoopPower);
            telemetry.update();
        }
    }
}
