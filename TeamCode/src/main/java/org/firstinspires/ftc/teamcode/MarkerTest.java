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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

@Autonomous(name="Marker Attachment", group="Test")
//@Disabled
public class MarkerTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorMarker = null;

    Servo servo3 = null;
    Servo servo4 = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorMarker     = hardwareMap.get(DcMotor.class, "motor5");
        motorMarker.setDirection(DcMotor.Direction.REVERSE);

        servo3  = hardwareMap.get(Servo.class, "servo3");
        servo4  = hardwareMap.get(Servo.class, "servo4");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        int origPosition = motorMarker.getCurrentPosition();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double power = 0;

            if (gamepad1.left_stick_x > 0)
                power = 0.50;
            else if (gamepad1.left_stick_x < 0)
                power = -0.50;
            motorMarker.setPower(power);


            if (gamepad1.b)
                setServoPosition(servo3, 1.0);

            if (gamepad1.a)
                setServoPosition(servo3, 0.25);

            if (gamepad1.x)
                setServoPosition(servo4, 1);

            if (gamepad1.y) //starting
                setServoPosition(servo4, 0.0);


            telemetry.addData("Motors", "#1 (%.2f)", power);
            telemetry.addData("Curr Pos", "Orig %d Curr %d Diff %d", origPosition, motorMarker.getCurrentPosition()
                                    , motorMarker.getCurrentPosition() - origPosition);

            telemetry.update();
        }
    }

    void setServoPosition(Servo servo, double targetPosition )
    {
        double currentPos = servo.getPosition();

        if (currentPos > targetPosition) {
            while (servo.getPosition() > targetPosition) {
                servo.setPosition( servo.getPosition() - 0.005);

                telemetry.addData("servo", servo.getPosition());
                telemetry.update();

            }
        }
        else if (currentPos < targetPosition) {
            while (servo.getPosition() < targetPosition) {

                servo.setPosition( servo.getPosition() + 0.005);

                telemetry.addData("servo", servo.getPosition());
                telemetry.update();

            }
        }
    }


}

