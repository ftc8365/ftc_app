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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


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

@TeleOp(name="DriverControl", group="TeleOp")
//@Disabled
public class DriverControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorFrontRight     = null;
    private DcMotor motorFrontLeft      = null;
    private DcMotor motorCenter         = null;
    private DcMotor motorLift           = null;
    private DcMotor motorIntakeHopper   = null;
    private DcMotor motorIntakeSlide    = null;
    Servo servo1 = null;
    Servo servo2 = null;


    double SCALING_FACTOR               = 0.80;
    boolean forwardFacing               = true;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motor2");
        motorCenter     = hardwareMap.get(DcMotor.class, "motor3");
        motorLift       = hardwareMap.get(DcMotor.class, "motor4");
        motorIntakeHopper   = hardwareMap.get(DcMotor.class, "motor6");
        motorIntakeSlide    = hardwareMap.get(DcMotor.class, "motor7");

        servo1  = hardwareMap.get(Servo.class, "servo1");
        servo2  = hardwareMap.get(Servo.class, "servo2");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorCenter.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Starting phone servo position
        servo1.setPosition(1);
        servo2.setPosition(0.38);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        this.forwardFacing = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() ) {

            operateLiftMotor();

            operateIntake();

            operatorDriveTrain();

            telemetry.update();
        }
    }


    void operatorDriveTrain() {

        if (gamepad1.y)
            this.forwardFacing = true;
        if (gamepad1.a)
            this.forwardFacing = false;

        double multiplier = this.forwardFacing ? 1 : -1;

        double motorFrontRightPower = 0;
        double motorFrontLeftPower = 0;
        double motorCenterPower = 0;

        if (gamepad1.right_trigger != 0)
            SCALING_FACTOR = 1.0;
        else
            SCALING_FACTOR = 0.80;


        double x1value = gamepad1.left_stick_x;
        if (Math.abs(x1value) > 0.1)
        {
            motorFrontRightPower = x1value * -1 * SCALING_FACTOR;
            motorFrontLeftPower = x1value * 1 * SCALING_FACTOR;
            motorCenterPower = x1value * -0.5 * SCALING_FACTOR;
        }
        else
        {
            int joystickPostion = getJoystickPosition();
            telemetry.addData("joystick pos", joystickPostion);

            switch (joystickPostion)
            {
                case 0:
                    motorCenterPower = 0;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    break;
                case 2:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier ; //* SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_x * 1 * multiplier ; //* SCALING_FACTOR;
                    motorFrontRightPower = 0.10;
                    break;
                case 4:
                    motorCenterPower = gamepad1.right_stick_x  * multiplier;
                    motorFrontRightPower = -0.10 * multiplier;
                    motorFrontLeftPower = 0.10 * multiplier;
                    break;
                case 6:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = -0.10;
                    break;
                case 8:
                    motorCenterPower = 0;
                    motorFrontRightPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier * SCALING_FACTOR;
                    break;
                case 10:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = gamepad1.right_stick_y * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = -0.10;
                    break;
                case 12:
                    motorCenterPower = gamepad1.right_stick_x * multiplier;
                    motorFrontRightPower = 0.10 * multiplier;
                    motorFrontLeftPower = -0.10 * multiplier;
                    break;
                case 14:
                    motorCenterPower = gamepad1.right_stick_x * 1 * multiplier; // * SCALING_FACTOR;
                    motorFrontRightPower = gamepad1.right_stick_x * -1 * multiplier; // * SCALING_FACTOR;
                    motorFrontLeftPower = 0.10;
                    break;
            }
        }

        motorFrontRight.setPower(motorFrontRightPower );
        motorFrontLeft.setPower(motorFrontLeftPower );
        motorCenter.setPower(motorCenterPower );

        telemetry.addData("Robot Facing", this.forwardFacing ? "FORWARD" : "BACKWARD");
        telemetry.addData("gamepad1.right_stick_y", gamepad1.right_stick_y);
        telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);

        telemetry.addData("motorFrontRightPower", motorFrontRightPower);
        telemetry.addData("motorFrontLeftPower", motorFrontLeftPower);
        telemetry.addData("motorCenterPower", motorCenterPower);
    }

    int getJoystickPosition()
    {
        int pos = 0;
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        if (x >= -0.1 && x < 0.1)
        {
            if (y < -0.75)
                return 0;

            if ( y > 0.75)
                return 8;
        }

        if (x >= 0.10 && x < 0.90)
        {
            if (y >= -0.90 && y <= -0.10)
                return 2;

            if (y <= 0.90 && y >= 0.10)
                return 6;
        }

        if (x <= -0.10 && x >= -0.90)
        {
            if (y >= -0.90 && y <= -0.10)
                return 14;

            if (y <= 0.90 && y >= 0.10)
                return 10;
        }

        if (x >= 0.90)
        {
            return 4;
        }

        if (x <= -0.90)
        {
            return 12;
        }





        return pos;
    }

    void operateLiftMotor() {

        double motorRPPower = 0;

        if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
            if (gamepad2.right_stick_y > 0.5)
                motorRPPower = 1.0;

            if (gamepad2.right_stick_y < -0.5)
                motorRPPower = -1.0;
        }

        motorLift.setPower(motorRPPower);

        telemetry.addData("motorLift", motorRPPower);
    }


    void operateIntake() {

        double powerSlide = 0;
        double powerHopper = 0;

        if ((gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) == false)
        {
            if (gamepad2.right_stick_y > 0)
                powerSlide = 0.8;
            else if (gamepad2.right_stick_y < 0)
                powerSlide = -0.8;

            motorIntakeSlide.setPower(powerSlide);
        }

        if (gamepad2.left_trigger > 0)
            powerHopper = 1.0;

        if (gamepad2.right_trigger > 0 )
            powerHopper = -1.0;

        motorIntakeHopper.setPower(powerHopper);

        telemetry.addData("motorIntakeSlide", powerSlide);
        telemetry.addData("motorIntakeHopper", powerHopper);

    }

}
