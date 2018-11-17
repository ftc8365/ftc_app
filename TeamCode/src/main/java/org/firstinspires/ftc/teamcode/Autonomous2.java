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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


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

@Autonomous(name="Autonomous Position 2", group="Autonomous")
//@Disabled
public class Autonomous2 extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime runtime = new ElapsedTime();

    /////////////////////
    // Declare motors
    /////////////////////
    DcMotor motorFrontRight     = null;
    DcMotor motorFrontLeft      = null;
    DcMotor motorCenter         = null;
    DcMotor motorLift           = null;
    DcMotor motorMarker         = null;
    DcMotor motorIntakeHopper   = null;
    DcMotor motorIntakeSlide    = null;

    double motorFrontRightPower = 0;
    double motorFrontLeftPower  = 0;
    double motorCenterPower     = 0;
    double motorLiftPower       = 0;
    int origMarkerPosition      = 0;

    enum GoldMineralPosition
    {
        NOT_FOUND,
        FOUND_LEFT_POSITION,
        FOUND_CENTER_POSITION,
        FOUND_RIGHT_POSITION
    };

    /////////////////////
    // Declare sensors
    /////////////////////
    ModernRoboticsI2cRangeSensor    rangeSensorBottom   = null;
//    ModernRoboticsI2cRangeSensor    rangeSensorFront    = null;

    // The IMU sensor object
    BNO055IMU                       imu;
    Orientation                     angles;
    Acceleration                    gravity;

    DigitalChannel digitalTouch;  // Hardware Device Object

    /////////////////////
    // Declare servos
    /////////////////////
    Servo servo1 = null;
    Servo servo2 = null;
    Servo servo3 = null;
    Servo servo4 = null;

    /////////////////////
    // Declare OpenCV
    /////////////////////
    private static final String VUFORIA_KEY = "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (4) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible = false;
    Dogeforia vuforia = null;
    WebcamName webcamName = null;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    GoldAlignDetector goldDetector;


    //////////////////////////////////////////////////////////////////////
    // End of  OpMode members declaration
    //////////////////////////////////////////////////////////////////////


    @Override
    public void runOpMode() {

        initMotors();
        initRangeSensors();
        initServos();
        initGyroSensor();
        initOpenCV();

        // Set up our telemetry dashboard
        composeTelemetry();

        // Starting phone servo position
        setPhoneStartingPostion();

        telemetry.addData("range_sensor", rangeSensorBottom.rawUltrasonic());

        waitForStart();

        runtime.reset();
        //Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        lowerRobot();


        turnLeftTillDegrees(30, 0.40);
        driveForwardRotation(1.6,0.40);

        turnRightTillDegrees(-5, 0.40);

        driveLeftTillRotation(0.50, 0.4);

        setPhoneScanPosition();

        sleep(1000);

        goldDetector.reset();

        boolean cont = true;

        double rotationToGo = 3.6;
        double rotationMoved = 0.0;
        boolean goldMineralFound = false;
        GoldMineralPosition goldMineralPos = GoldMineralPosition.NOT_FOUND;

        while (opModeIsActive() && cont)
        {
            if (goldMineralPos != GoldMineralPosition.NOT_FOUND)
            {
                switch ( goldMineralPos )
                {
                    case FOUND_LEFT_POSITION:
                        this.driveForwardRotation(1.0, 0.50);
                        break;

                    case FOUND_CENTER_POSITION:
                        this.driveForwardRotation(1.0, 0.50);
                        break;

                    case FOUND_RIGHT_POSITION:
                        this.driveForwardRotation(1.0, 0.50);
                        break;
                }

                this.driveForwardTillTime(1, 0.40);
                extendIntakeTillTime(1, 0.35);

                cont = false;
            }
            else
            {
                goldMineralPos = driveRightTillGoldAligned( rotationToGo, 0.45);

                if (goldMineralPos == GoldMineralPosition.NOT_FOUND)
                    goldMineralPos = GoldMineralPosition.FOUND_RIGHT_POSITION;

                setPhoneStartingPostion();
            }
        }



/*
        driveRightTillRotation(1.5,0.40);

        turnRightTillDegrees(35,0.35);

        telemetry.addData("range", rangeSensorFront.rawUltrasonic());
        telemetry.update();

        driveForwardTillRange( 16, 0.25);
        telemetry.addData("range", rangeSensorFront.rawUltrasonic());
        telemetry.update();


        turnRightTillDegrees(120, .35);

        driveForwardTillTime(3, .35);

        extendIntakeTillTime(2, 0.35)
*/

        vuforia.stop();
    }

    void setPhoneStartingPostion()
    {
        servo1.setPosition(1.00);
        servo2.setPosition(0.38);
    }

    void setPhoneScanPosition() {
        // Rotate servo to positions photo in the front tiled
        setServoPosition(servo2, 0.55);
        setServoPosition(servo1, 0.40);
    }

    void lowerRobot() {

        while (rangeSensorBottom.rawUltrasonic() > 6) {
            telemetry.addData("range", rangeSensorBottom.rawUltrasonic());
            telemetry.update();
            motorLift.setPower(0.4);
        }

        motorLift.setPower(0.0);
        sleep(150);
    }


    void lowerRobot1()
    {
        if (rangeSensorBottom.rawUltrasonic() >= 6) {
            int currentRange = rangeSensorBottom.rawUltrasonic();
            int counter = 0;

            motorLift.setPower(0.5);

            while (rangeSensorBottom.rawUltrasonic() >= 6) {
                ++counter;

                if (counter >= 50) {
                    if (rangeSensorBottom.rawUltrasonic() >= currentRange)
                        break;
                    else
                        currentRange = rangeSensorBottom.rawUltrasonic();
                    counter = 0;
                }
            }
            sleep(100);
            motorLift.setPower(0);
        }
        telemetry.addData("range", rangeSensorBottom.rawUltrasonic());
        telemetry.update();

    }

    void extendIntakeTillTime(double seconds, double targetPower)
    {
        motorIntakeSlide.setPower( targetPower );

        sleep( (long)(seconds * 1000) );

        motorIntakeSlide.setPower(0);
    }

    void grabMineral1() {
        setPhoneStartingPostion();
        driveForwardRotation(0.35, 0.35);
        sleep(1500);
        driveBackwardRotation(0.35, 0.35);
    }

    void ejectMineral()
    {
        setPhoneStartingPostion();

        motorIntakeHopper.setPower(1.0);

        sleep(1500);

        motorIntakeHopper.setPower(0.0);
    }

    void grabMineral()
    {
        setPhoneStartingPostion();

        int SLIDING_DISTANCE = 2000;

        int startIntakePos = motorIntakeSlide.getCurrentPosition();

        while (startIntakePos - motorIntakeSlide.getCurrentPosition() < SLIDING_DISTANCE)
            motorIntakeSlide.setPower(-0.50);

        motorIntakeSlide.setPower(0);

        motorIntakeHopper.setPower(-1.0);

        sleep(1500);

        motorIntakeHopper.setPower(0);

        motorIntakeSlide.setPower(0.5);

        while (startIntakePos - motorIntakeSlide.getCurrentPosition() > 50)
            motorIntakeSlide.setPower(0.5);

        motorIntakeSlide.setPower(0);

    }

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

    private void initMotors() {
        motorFrontRight     = hardwareMap.get(DcMotor.class, "motor1");
        motorFrontLeft      = hardwareMap.get(DcMotor.class, "motor2");
        motorCenter         = hardwareMap.get(DcMotor.class, "motor3");
        motorLift           = hardwareMap.get(DcMotor.class, "motor4");
        motorMarker         = hardwareMap.get(DcMotor.class, "motor5");
        motorIntakeHopper   = hardwareMap.get(DcMotor.class, "motor6");
        motorIntakeSlide    = hardwareMap.get(DcMotor.class, "motor7");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorCenter.setDirection(DcMotor.Direction.FORWARD);
        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorMarker.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initRangeSensors() {
        rangeSensorBottom   = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor1");
//        rangeSensorFront    = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"range_sensor2");

 //       digitalTouch = hardwareMap.get(DigitalChannel.class, "touch1");
   //     digitalTouch.setMode(DigitalChannel.Mode.INPUT);

    }

    private void initServos() {
        servo1  = hardwareMap.get(Servo.class, "servo1");
        servo2  = hardwareMap.get(Servo.class, "servo2");
        servo3  = hardwareMap.get(Servo.class, "servo3");
        servo4  = hardwareMap.get(Servo.class, "servo4");
    }


    public void initGyroSensor() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private void initOpenCV() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        goldDetector = new GoldAlignDetector();
        goldDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);

        goldDetector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        goldDetector.useDefaults();
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        goldDetector.setAlignSettings( 0, 100);

        vuforia.setDogeCVDetector(goldDetector);

        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        /*
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
        telemetry.addLine()
                .addData("Motor LFT", "(%.2f)", motorLiftPower);
*/

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
    void driveForwardRotation( double rotation, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );
        motorCenter.setPower( 0.0 );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    void driveForwardTillTime( double seconds, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.10;

        motorFrontRight.setPower( power );
        motorFrontLeft.setPower( power );
        motorCenter.setPower( 0.0 );

        while (power < targetPower)
        {
            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        sleep( (long)(seconds * 1000));

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    /*
    void driveForwardTillRangeX( int range, double targetPower )
    {
        double power = 0.05;
        boolean cont = true;

        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorCenter.setPower( 0.0 );

        while (cont)
        {
            if (rangeSensorFront.rawUltrasonic() < range)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power );
            motorFrontLeft.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }
*/


    ////////////////////////////////////////////////////////
    void driveRightTillRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition >= 1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( -0.20 );
            motorFrontLeft.setPower( 0.20 );
            motorCenter.setPower( power );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }


    ////////////////////////////////////////////////////////
    void driveLeftTillRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( 0.10 );
            motorFrontLeft.setPower( -0.10 );
            motorCenter.setPower( power * -1 );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ////////////////////////////////////////////////////////
    GoldMineralPosition driveRightTillGoldAligned( double maxRotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();
        int curPosition = initPosition;
        GoldMineralPosition pos = GoldMineralPosition.NOT_FOUND;

        double power = 0.05;

        motorFrontRight.setPower( 0 );
        motorFrontLeft.setPower( 0 );

        while (true)
        {
            if (goldDetector.isAligned())
            {
                double distance = curPosition - initPosition;

                if (distance < 500)
                    pos = GoldMineralPosition.FOUND_LEFT_POSITION;
                else if (distance >= 500 && distance < 2500)
                    pos = GoldMineralPosition.FOUND_CENTER_POSITION;
                else
                    pos = GoldMineralPosition.FOUND_RIGHT_POSITION;
                break;

            }

            curPosition = motorCenter.getCurrentPosition();
            if (curPosition - initPosition >= 1000 * maxRotation)
            {
                pos = GoldMineralPosition.FOUND_RIGHT_POSITION;
                break;
            }

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower(-0.20 );
            motorFrontLeft.setPower( 0.20 );
            motorCenter.setPower( power * 1 );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

        telemetry.addData("pos", curPosition - initPosition);
        telemetry.addData("enum", pos);
        telemetry.update();

        return pos;
    }


    ////////////////////////////////////////////////////////
    void driveBackwardRotation( double rotation, double targetPower )
    {
        int initPosition = motorFrontRight.getCurrentPosition();

        boolean cont = true;
        double power = 0.05;

        motorCenter.setPower( 0.0 );

        while (cont)
        {
            if (motorFrontRight.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.02;

            motorFrontRight.setPower( power * -1 );
            motorFrontLeft.setPower( power * -1 );
        }

        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);
    }



    ////////////////////////////////////////////////////////
    void turnRightRotation( double rotation, double targetPower )
    {
        int initPosition = motorCenter.getCurrentPosition();

        boolean cont = true;
        double power = 0.10;

        while (cont)
        {
            if (motorCenter.getCurrentPosition() - initPosition <= -1000 * rotation)
                cont = false;

            if (power < targetPower)
                power += 0.01;

            motorFrontRight.setPower(power * -1);
            motorFrontLeft.setPower(power);
            motorCenter.setPower(power * -1);
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

    void turnRightTillDegrees( int targetDegrees, double targetPower )
    {
        double startHeading = getCurrentHeadingRightTurn();

        int offset = 0;
        double power = 0.05;

        boolean continueToTurn = true;

        while ( continueToTurn )
        {
            double currentHeading = getCurrentHeadingRightTurn();

            if ( ( (currentHeading + offset ) >= targetDegrees ))
            {
                continueToTurn = false;
            }
            else
            {
                if (power < targetPower)
                    power += 0.02;

                motorFrontRight.setPower( power * -1 );
                motorFrontLeft.setPower( power );
                motorCenter.setPower( power * -0.50 ) ;
            }
        }

        // Stop motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

    }
    ///////////////////////////////////////////////////////////////////////////////////////////

    void turnLeftTillDegrees( int targetDegrees, double targetPower )
    {
        double startHeading = getCurrentHeadingLeftTurn();

        int offset = 0;

        boolean continueToTurn = true;
        double power = 0.05;

        while ( continueToTurn )
        {
            double currentHeading = getCurrentHeadingLeftTurn();

            if ( ( (currentHeading + offset ) >= targetDegrees ))
            {
                continueToTurn = false;
            }
            else
            {
                if (power < targetPower)
                    power += 0.02;

                motorFrontRight.setPower(power );
                motorFrontLeft.setPower(power * -1 );
                motorCenter.setPower(power * 0.5) ;
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

    float getCurrentHeadingLeftTurn() {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
///////////////////////////////////////////////////////////////////////////////////////////

    void turnRightTillAlign( int targetDegrees, double power )
    {
        int offset = 0;

        boolean continueToTurn = true;

        while ( continueToTurn )
        {
            if (goldDetector.isAligned())
                continueToTurn = false;

            double multiplier = 1;

            motorFrontRight.setPower(power * -1 * multiplier);
            motorFrontLeft.setPower(power * multiplier);
            motorCenter.setPower(power * -1 * multiplier) ;
        }

        // Stop motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorCenter.setPower(0);

    }

    void lowerGameMarker() {
        origMarkerPosition = motorMarker.getCurrentPosition();

        motorMarker.setPower(0.70);

        while (motorMarker.getCurrentPosition() - origMarkerPosition < 1000)
            motorMarker.setPower(0.70);

        setServoPosition(servo3, 0.35);

        while (motorMarker.getCurrentPosition() - origMarkerPosition < 4000)
            motorMarker.setPower(0.70);

        motorMarker.setPower(0.00);

        setServoPosition(servo4, 1.0);

        sleep(1000);
    }

    void raiseGameMarker()
    {
        motorMarker.setPower(-1.0);

        while (motorMarker.getCurrentPosition() - origMarkerPosition > 3500)
            motorMarker.setPower(-1.00);

        motorMarker.setPower(-0.40);

        servo4.setPosition(0);

        servo3.setPosition(0.25);

        while (motorMarker.getCurrentPosition() - origMarkerPosition > 100)
            motorMarker.setPower(-0.40);

        motorMarker.setPower(0);

    }


    void lowerGameMarkerX() {
        this.driveForwardRotation(0.35, 0.35);

        origMarkerPosition = motorMarker.getCurrentPosition();

        motorMarker.setPower(0.70);

        while (motorMarker.getCurrentPosition() - origMarkerPosition < 1000)
            motorMarker.setPower(0.70);

        setServoPosition(servo3, 0.95);

        while (motorMarker.getCurrentPosition() - origMarkerPosition < 10000)
            motorMarker.setPower(0.70);

        motorMarker.setPower(0.00);

        setServoPosition(servo4,1.0);

        sleep(500);
    }

    void raiseGameMarkerX()
    {
        motorMarker.setPower(-1.00);

        while (motorMarker.getCurrentPosition() - origMarkerPosition > 3000)
            motorMarker.setPower(-1.00);

        motorMarker.setPower(-0.40);

        servo4.setPosition(0);

        servo3.setPosition(0.25);

        while (motorMarker.getCurrentPosition() - origMarkerPosition > 100)
            motorMarker.setPower(-0.40);

        motorMarker.setPower(0);

        this.driveBackwardRotation(0.35, 0.35);

    }

    void setServoPosition( Servo servo, double targetPosition )
    {
        double currentPos = servo.getPosition();

        if (currentPos > targetPosition) {
            while (servo.getPosition() > targetPosition) {
                servo.setPosition( servo.getPosition() - 0.1);

            }
        }
        else if (currentPos < targetPosition) {
            while (servo.getPosition() < targetPosition) {

                servo.setPosition( servo.getPosition() + 0.1);

                telemetry.addData("servo", servo.getPosition());
                telemetry.update();

            }
        }
    }

}
