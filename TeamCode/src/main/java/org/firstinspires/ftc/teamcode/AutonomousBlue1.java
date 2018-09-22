/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "AutonomousBlue1", group = "Autonomous")
//@Disabled

public class AutonomousBlue1 extends LinearOpMode
{

    /////////////////////////////////////////////////////////
    // Declare all motor & sensors here
    /////////////////////////////////////////////////////////
    DcMotor                         motorFR         = null;
    DcMotor                         motorFL         = null;
    DcMotor                         motorBR         = null;
    DcMotor                         motorBL         = null;
//    DcMotor                         LEDLight        = null;
    ModernRoboticsI2cRangeSensor    rangeSensor     = null;
    ColorSensor                     colorSensor     = null;
    VuforiaLocalizer                vuforia;
    ModernRoboticsI2cGyro           gyroSensor      = null;

    DcMotor                         motorRPFront    = null;
    Servo                           servoJewel      = null;
    Servo                           servoR1         = null;
    Servo                           servoL1         = null;

    double                          glyphRotation   = 0;


    @Override
    public void runOpMode()  throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */

        parameters.vuforiaLicenseKey = "ATYC0pz/////AAAAGYlRzOAzM0Sup56lImTVThdHStSVQMRWU4oNVkFjXnwX1IvAA1sipEHJjzZBS3pmFkqsuT8q0YXYhOJ4cQHrDySFRvBJgUrEymr8q2j91kwm7to94MmFWgKsecydly+SNsYSBPNk3exX0G8p81ybbuX2zcVFJEZtfKRKmig0feeDNMEuEdJuFdG6tS+o80CRtL4e/T8KhEG09PyP6ZXPIaaFpIy7WFszGz438yv3OVoyK9A0qeNsl5ZTDM9S9j5PKKxDXPW1cOddBOM544LDFf7F9noqZ+XZgQSGwn0jFhYXBYbur35ezAA+dh/ke3bTs4iTvFwL/6PCWfQz68nGqa26ev512QEhPlIi5NDexUWH"; // Insert your own key here

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        ////////////////////////////////////////////////////////////
        // Initialize Robot
        ////////////////////////////////////////////////////////////
        servoJewel = hardwareMap.get(Servo.class, "servo1");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        telemetry.addData("range_sensor", "initialized");

        // get a reference to the color sensor.
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        telemetry.addData("color_sensor", "initialized");

        colorSensor.enableLed(true);

//        LEDLight = hardwareMap.dcMotor.get("LED");
//        telemetry.addData("LED Light", "initialized");

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFR.setDirection( DcMotor.Direction.FORWARD );
        motorFR.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection( DcMotor.Direction.REVERSE );
        motorFL.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBR.setDirection( DcMotor.Direction.FORWARD );
        motorBR.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );


        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection( DcMotor.Direction.REVERSE );
        motorBL.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

        telemetry.addData("motorFR", "initialized");
        telemetry.addData("motorFL", "initialized");
        telemetry.addData("motorBR", "initialized");
        telemetry.addData("motorBL", "initialized");

        int initPosition = motorFR.getCurrentPosition();
        telemetry.addData("motorFR initPos", initPosition);

        servoR1 = hardwareMap.get(Servo.class, "servoR1");
        servoL1 = hardwareMap.get(Servo.class, "servoL1");

        motorRPFront = hardwareMap.dcMotor.get("motorRP");
        motorRPFront.setDirection(DcMotor.Direction.REVERSE);
        motorRPFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRPFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.update();

        ////////////////////////////////////////////////////////////
        // wait for the start button to be pressed
        ////////////////////////////////////////////////////////////

        waitForStart();

        // Lower Jewel servo
        servoJewel.setPosition(0.1);

//        LEDLight.setPower(-0.25);

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark;
        boolean vuMarkFound = false;
        int vuMarkScanCount = 0;

        while (vuMarkFound == false && ++vuMarkScanCount < 200 )
        {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark == RelicRecoveryVuMark.LEFT)
            {
                vuMarkFound = true;

                this.glyphRotation = 0.30;
            }
            else if (vuMark == RelicRecoveryVuMark.CENTER)
            {
                vuMarkFound = true;

                this.glyphRotation = 1.45;
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT)
            {
                vuMarkFound = true;

                this.glyphRotation = 2.65;
            }
            else
            {
                vuMarkFound = false;
                this.glyphRotation = 1.45;

                sleep(10);
            }
        }

        telemetry.addData("Glyph Position", glyphRotation);
        telemetry.update();


        // Close the Glyph attachment Arms
        servoL1.setPosition(0.0);
        servoR1.setPosition(1.0);

        sleep(1000);

        // Raise the front Rack & Pinion
        motorRPFront.setPower(0.35);
        sleep(1000);

        motorRPFront.setPower(0.00);
        motorRPFront.setPower(0.00);

        if ( isBlue( colorSensor.red(), colorSensor.green(), colorSensor.blue()) )
        {
            driveForwardRotation(1.0, 0.15);

            telemetry.addData("See Blue", "TRUE");
        }
        else
        {
            driveBackwardRotation(0.20, 0.15);
            telemetry.addData("See Blue", "FALSE");
        }

        telemetry.update();

        // Raise Jewel Servo
        servoJewel.setPosition(0.7);
        sleep(1000);

        driveForwardTillRange(35, 0.30);

        driveBackwardRotation(0.25, 0.30);

        driveRightTillRotation(glyphRotation, 0.40);

        // Open the Glyph attachment Arms
        servoL1.setPosition(1.0);
        servoR1.setPosition(0.0);

        // Lower the front Rack & Pinion
        motorRPFront.setPower(-0.35);
        sleep(500);
        motorRPFront.setPower(0);

        goForwardUntilTime( 2000, 0.20 );

        driveBackwardRotation(0.30, 0.30);

//        LEDLight.setPower(0.0);
    }

    ////////////////////////////////////////////////////////
    void driveForwardRotation( double rotation, double power )
    {
        int initPosition = motorBR.getCurrentPosition();

        boolean cont = true;

        motorFR.setPower( power );
        motorFL.setPower( power );
        motorBR.setPower( power );
        motorBL.setPower( power );

        while (cont)
        {
            telemetry.addData("motorFR curPos", motorFR.getCurrentPosition());
            telemetry.addData("motorBR curPos", motorBR.getCurrentPosition());
            telemetry.update();

            if (motorBR.getCurrentPosition() - initPosition > 950 * rotation)
                cont = false;
        }

        stopMotors();
    }


    ////////////////////////////////////////////////////////
    void driveBackwardRotation( double rotation, double power )
    {
        int initPosition = motorBR.getCurrentPosition();

        boolean cont = true;

        motorFR.setPower( -1 * power );
        motorFL.setPower( -1 * power );
        motorBR.setPower( -1 * power );
        motorBL.setPower( -1 * power );

        while (cont)
        {
            telemetry.addData("motorFR curPos", motorFR.getCurrentPosition() );
            telemetry.addData("motorBR curPos", motorBR.getCurrentPosition() );
            telemetry.update();

            if (  motorBR.getCurrentPosition() - initPosition < -950 * rotation )
                cont = false;
        }

        stopMotors();

    }



    ////////////////////////////////////////////////////////
    void driveRightTillRotation( double rotation, double power )
    {
        int initPosition = motorBR.getCurrentPosition();

        boolean cont = true;

        motorFR.setPower( -1 * power );
        motorFL.setPower( power );
        motorBR.setPower( power );
        motorBL.setPower( -1 * power );

        while (cont)
        {
            if (motorBR.getCurrentPosition() - initPosition > (950 * rotation))
                cont = false;
        }

        stopMotors();

    }

    ////////////////////////////////////////////////////////
    void driveLeftTillRotation( double rotation, double power )
    {
        int initPosition = motorBR.getCurrentPosition();

        boolean cont = true;

        motorFR.setPower( power );
        motorFL.setPower(-1 * power );
        motorBR.setPower(-1 * power );
        motorBL.setPower( power );

        while (cont)
        {
            if (initPosition - motorBR.getCurrentPosition() > (950 * rotation))
                cont = false;
        }

        stopMotors();

    }

    ////////////////////////////////////////////////////////
    boolean isBlue( int red, int green, int blue )
    {
        boolean isBlue = false;

        if ( blue > red && blue > green )
            isBlue = true;

        return isBlue;
    }

    ////////////////////////////////////////////////////////
    boolean isRed( int red, int green, int blue )
    {
        boolean isRed = false;

        if ( red > blue && red > green )
            isRed = true;

        return isRed;
    }



    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    void driveForwardTillRange( int range, double power )
    {
        driveTillRange( true, range, power );
    }

    void driveBackwardTillRange( int range, double power )
    {
        driveTillRange( false, range, power );
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////
    void driveTillRange( boolean forward, int range, double power )
    {
        boolean rangeReached = false;
        int multiplier = forward ? 1 : -1;

        if ( forward == true && rangeSensor.rawUltrasonic() <= range )
        {
            rangeReached = true;
        }

        if ( forward == false && rangeSensor.rawUltrasonic() >= range )
        {
            rangeReached = true;
        }

        while ( rangeReached == false )
        {
            motorFL.setPower(multiplier * power);
            motorFR.setPower(multiplier * power);
            motorBL.setPower(multiplier * power);
            motorBR.setPower(multiplier * power);

            if ( rangeSensor.rawUltrasonic() <= range )
            {
                rangeReached = true;
            }
        }

        stopMotors();
    }

    ////////////////////////////////////////////////////////
    void goForwardUntilTime( int timeInMillisecond, double power )
    {
        boolean cont = true;

        motorFR.setPower( power );
        motorFL.setPower( power );
        motorBR.setPower( power );
        motorBL.setPower( power );

        sleep( timeInMillisecond );

        stopMotors();

    }



    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    void turnRightTillDegrees( int targetDegrees, double power )
    {
        turnRightTillDegrees( targetDegrees, power, 0 );
    }

    void turnRightTillDegrees( int targetDegrees, double power, int tolerance )
    {
        int startHeading = gyroSensor.getHeading();

        int offset = 0;

        if ( targetDegrees < 240 )
            targetDegrees += 360;

        if ( startHeading < 240 )
        {
            offset = 360;
        }

        boolean continueToTurn = true;

        while ( continueToTurn )
        {
            int currentHeading = gyroSensor.getHeading();

            if ( currentHeading < 240 && offset == 0)
                offset = 360;

            if ( ( (currentHeading + offset - tolerance ) >= targetDegrees ))
            {
                continueToTurn = false;
            }
            else
            {
                double multiplier = 1;

                motorFL.setPower(       power * multiplier );
                motorFR.setPower( -1 *  power * multiplier );
                motorBL.setPower(       power * multiplier );
                motorBR.setPower( -1 *  power * multiplier );
            }
        }

        stopMotors();

    }


    ///////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    void turnLeftTillDegrees( int targetDegrees, double power )
    {
        turnLeftTillDegrees( targetDegrees, power, 0 );
    }

    void turnLeftTillDegrees( int targetDegrees, double power, int tolerance )
    {
        int startHeading = gyroSensor.getHeading();

        int offset = 0;

        if ( targetDegrees < 180 )
            targetDegrees += 360;

        if ( startHeading < 180 )
        {
            offset = 360;
        }

        boolean continueToTurn = true;

        while ( continueToTurn )
        {
            int currentHeading = gyroSensor.getHeading();

            if ( currentHeading > 180 && offset == 360)
                offset = 0;

            if ( ( (currentHeading + offset - tolerance ) <= targetDegrees ))
            {
                continueToTurn = false;
            }
            else
            {
                double multiplier = 1;

                motorFL.setPower( -1 * power * multiplier );
                motorFR.setPower(      power * multiplier );
                motorBL.setPower( -1 * power * multiplier );
                motorBR.setPower(      power * multiplier );
            }
        }

        stopMotors();

    }



    void stopMotors()
    {
        for (int i = 0;  i < 3; ++i)
        {
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);
        }
    }


}
