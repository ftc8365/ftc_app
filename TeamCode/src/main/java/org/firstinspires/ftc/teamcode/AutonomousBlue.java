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
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
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

@TeleOp(name="AutonomousBlue", group="Test")
//@Disabled
public class AutonomousBlue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorFrontRight = null;
    private DcMotor motorFrontLeft  = null;
    private DcMotor motorCenter     = null;
    ModernRoboticsI2cRangeSensor rangeSensor    = null;

    private Servo servo1 = null;
    private Servo servo2 = null;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    boolean isButtonPressed = false;
    double motorFrontRightPower = 0;
    double motorFrontLeftPower  = 0;
    double motorCenterPower     = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("range_sensor1","initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontRight = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motor2");
        motorCenter     = hardwareMap.get(DcMotor.class, "motor3");
        rangeSensor     = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor1");

        servo1  = hardwareMap.get(Servo.class, "servo1");
        servo2  = hardwareMap.get(Servo.class, "servo2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCenter.setDirection(DcMotor.Direction.FORWARD);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //driveForwardRotation(1,0.5);
        //driveRightRotation(2,0.5);
        //turnRightRotation(1.5,0.5);
        //turnRightTillDegrees(45, 0.25);

       // servo1.setPosition(0);
        servo2.setPosition(1);
        sleep(2000);
        servo1.setPosition(0);




        sleep(1000000);
//        driveForwardTillRange(4, 50);

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });


        // Show the elapsed game time and wheel power.
        telemetry.addLine()
            .addData("Motor FR", "(%.2f)", motorFrontRightPower);
        telemetry.addLine()
                .addData("Motor FL", "(%.2f)", motorFrontLeftPower);
        telemetry.addLine()
                .addData("Motor CTR", "(%.2f)", motorCenterPower);

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    ////////////////////////////////////////////////////////
    void driveForwardRotation( double rotation, double power )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
    }



    ////////////////////////////////////////////////////////
    void driveForwardTillRange( double range, double power )
    {

        boolean cont = true;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );

        while (cont)
        {
            if (rangeSensor.rawUltrasonic() <= 10)
                cont = false;
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
    }



    ////////////////////////////////////////////////////////
    void driveBackwardRotation( double rotation, double power )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower( power * -1 );
        motorFrontLeft.setPower( power * -1);

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
    }


    ////////////////////////////////////////////////////////
    void driveRightRotation( double rotation, double power )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(power);

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);


    }

    ////////////////////////////////////////////////////////
    void driveLeftRotation( double rotation, double power )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(power);

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;
        }

        //motorFrontRight.setPower(0);
        //motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    void turnRightRotation( double rotation, double power )
    {
        int initPosition = motorCenter.getCurrentPosition();


        boolean cont = true;

        motorFrontRight.setPower(power * -1);
        motorFrontLeft.setPower(power);
        motorCenter.setPower(power * -1);

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    void turnLeftRotation( double rotation, double power )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power * -1);
        motorCenter.setPower(power * 1);

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    void turnRightTillDegrees( int targetDegrees, double power )
    {
        double startHeading = getCurrentHeadingRightTurn();

        int offset = 0;

//        if ( targetDegrees < 240 )
//            targetDegrees += 360;

//        if ( startHeading < 240 )
//        {
//            offset = 360;
//        }

        boolean continueToTurn = true;

        while ( continueToTurn )
        {
            double currentHeading = getCurrentHeadingRightTurn();

//            if ( currentHeading < 240 && offset == 0)
//                offset = 360;

            if ( ( (currentHeading + offset ) >= targetDegrees ))
            {
                continueToTurn = false;
            }
            else
            {
                double multiplier = 1;

                motorFrontRight.setPower(power * -1 * multiplier);
                motorFrontLeft.setPower(power * multiplier);
                motorCenter.setPower(power * -1 * multiplier) ;
            }
        }

        // Stop motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

    }


    float getCurrentHeadingRightTurn() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) *-1;
    }

}
