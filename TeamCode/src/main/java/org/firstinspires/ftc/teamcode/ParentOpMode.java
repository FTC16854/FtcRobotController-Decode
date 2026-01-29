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

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Original FTC opmode header block
 *
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
 **/

/** Parent OpMode Class
 * All Teleop and Autonomous OpModes should inherit from (extend) ParentOpMode.
 * Each child/subclass OpMode should have its own unique runOpMode() method that will
 * override the ParentOpMode runOpMode() method.
 **/

@TeleOp(name="Parent Opmode", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    //Drivetrain motors
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor spinMotor = null;

    //shotgun motor
    private DcMotorEx shotgunMotor = null;

    //intake motor
    private DcMotor rubberIntake = null;

    // Color Sensors
    private NormalizedColorSensor bulletIntakeColorSensor;
    private NormalizedColorSensor shotgunLoadingColorSensor;

    //spindexer hardware
    private CRServo TheKeeperOfTheBalls = null;
    private Servo shotgunTriggerServo = null;
    private Servo spindexServo = null;
    private DigitalChannel spindexPositionSwitch = null;

    //Parking System
    private Servo snailServo = null;

    //other Systems
    SparkFunOTOS OdometrySensor;

    //Other Global Variables
    SparkFunOTOS.Pose2D pos;
    //put global variables here...
    double servoPosition0 = 0.0128;      // The first position that the spindexer servo can turn to
    double servoPosition1 = 0.0870;   // The second position that the spindexer servo can turn to
    double servoPosition2 = 0.1578;   // The third position that the spindexer servo can turn to

    String[] colorArray = new String[3];    // The colors of the different balls in the spindexer
    double[] PosArray = {servoPosition0, servoPosition1, servoPosition2};   // The different positions that the spindexer servo can turn to

    double[] hackyPosArray =
            {
                    0.0130, //0
                    0.0870, //1
                    0.1578, //2
                    0.2328, //3
                    0.3083, //4
                    0.3822, //5
                    0.4583, //6
                    0.5400, //7
                    0.6106, //8
                    0.6867, //9
                    0.7600, //10
                    0.8339, //11
                    0.9028, //12
                    0.9739, //13
            };
    int colorPosIndex = 2;
    int hackyPosIndex = 2;
    double SpindexPosition = 0.0139;        // This is for the our hacky manual controls
    double SpindexIncrement = 0.07;//
    int SpindexMotorIncrement = (int) 1425.1 / 3;//==475 currently
    int spinnyGoHere = 0; // target position of the spindexer
    int spindexerArrayIndex = 0;            // For the color array, the index of the ball in the intake position
    int ShootgunIndex = 2;// For the color array, the index of the ball in the shooter position
    int shotgunSpeed = 1650;
    String tempBulletColor;
    boolean tempBulletolorIsSaved = false;
    boolean shotgunSpinyPB = false;
    boolean ToggleSpeed = false;
    boolean ToggleIntake = false;
    double triggerDown = 0;
    double triggerUp = 0.29; //0.255
    double snailPosition = 0.585; //0.450
    String snailDirection = "retracted";
    final float colorSensorGain = (float) 17.5;

    double spindexLockoutDelayStart = 0;   // Delay for spindexer after releasing trigger button
    double spindexLockoutDelay = 2000;    //milliseconds

    public void initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get()' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).

        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        shotgunMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        rubberIntake = hardwareMap.get(DcMotor.class, "intake");
        spinMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");

        TheKeeperOfTheBalls = hardwareMap.get(CRServo.class, "intakeServo");
        shotgunTriggerServo = hardwareMap.get(Servo.class, "triggerServo");
        spindexServo = hardwareMap.get(Servo.class, "spindexServo");
        snailServo = hardwareMap.get(Servo.class, "snail_servo");

        spindexPositionSwitch = hardwareMap.get(DigitalChannel.class, "spindex position switch");

        OdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        bulletIntakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        bulletIntakeColorSensor.setGain(colorSensorGain);
        shotgunLoadingColorSensor = hardwareMap.get(NormalizedColorSensor.class, "shotgun_color");
        shotgunLoadingColorSensor.setGain(colorSensorGain);
        //Set motor run mode (esp. if using SPARK Mini motor controller(s) for drivetrain)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shotgunMotor.setDirection(DcMotor.Direction.REVERSE);
        rubberIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        spinMotor.setDirection(DcMotor.Direction.FORWARD);

        spindexServo.setDirection(Servo.Direction.REVERSE);
        shotgunTriggerServo.setDirection(Servo.Direction.REVERSE);
        snailServo.setDirection(Servo.Direction.REVERSE);
        TheKeeperOfTheBalls.setDirection(CRServo.Direction.REVERSE);

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shotgunMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rubberIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Update Driver Station Status Message after init
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }

    /**
     * runOpMode() will be overridden in child OpMode.
     * Basic structure should remain intact (init, wait for start, while(opModeIsActive),
     * Additionally, Emergency conditions should be checked during every cycle
     */
    @Override
    public void runOpMode() {

        initialize();

        // Init loop - optional
        while (opModeInInit()) {
            telemetry.addData("Cheese and Pickles ", "Sandwich!");
            // Code in here will loop continuously until OpMode is started
        }

        // Wait for the game to start (driver presses PLAY) - May not be needed if using an init Loop
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes

            int spinmotortarget = 0;
            int spinmotorincrement = (int) 1425.1 / 3; //== 475 currently

            //include emergency stop check in all runOpMode() functions/methods
            //implementation depends on which E-stop function will be used (boolean/void)


            checkEmergencyStop(); // Stops motors and Terminates if buttons are pressed
            //without additional code in the while(opModeIsActive) loop.

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid accessing gamepad directly in other functions/methods
    //This also makes it simpler to re-map controls as desired
    //CONTROLLER MAP

    // Thumbsticks
    public double left_sticky_x() {
        return gamepad1.left_stick_x;
    }

    public double left_sticky_y() {
        return -gamepad1.left_stick_y;
    }

    public double right_sticky_y() {
        return -gamepad1.right_stick_y;
    }

    public double right_sticky_x() {
        return gamepad1.right_stick_x;
    }

    public boolean slowDownStick() {
        return gamepad1.leftStickButtonWasPressed();
    }

    // Buttons
    public boolean spindex_left() {
        return gamepad1.left_bumper;
    }

    public boolean spindex_left_was_pressed() {
        return gamepad2.leftBumperWasPressed();
    }

    public boolean spindex_left_was_Released() {
        return gamepad2.leftBumperWasReleased();
    }

    public boolean spindex_right() {
        return gamepad1.right_bumper;
    }

    public boolean spindex_right_was_pressed() {
        return gamepad2.rightBumperWasPressed();
    }

    public boolean spindex_right_was_released() {
        return gamepad2.rightBumperWasReleased();
    }

    public boolean shotgunToggleSpinyPB() {
        return gamepad2.aWasPressed();
    }

    public boolean spindexerResetPB() {
        return gamepad2.dpad_up && gamepad2.y;
    }

    public boolean shotgunTriggerPB() {
        return gamepad2.right_trigger > 0.5;
    }

    public boolean shotgunTriggerWasReleased(){
        return gamepad2.rightTriggerWasReleased();
    }

    public boolean rubberToggleIntakePB() {
        return gamepad1.bWasPressed();
    }

    public boolean rubberIntakePB() {
        return gamepad1.right_trigger > 0.5;
    }

    public boolean rubberOuttakePB() {
        return gamepad1.right_bumper;
    }

    public boolean FieldCentricReset() {
        return gamepad1.back || gamepad2.back;
    }

    public boolean emergencyButtons() {
        // check for combination of buttons to be pressed before returning true
        return (gamepad1.b && gamepad1.y) || (gamepad2.b && gamepad2.y);
    }

    public boolean SnailPB_was_Released() {
        return gamepad1.dpadUpWasPressed();
    }

    public boolean IncrementorPlusButton() {
        return gamepad2.dpad_up;
    }

    public boolean IncrementorMinusButton() {
        return gamepad2.dpad_down;
    }

    public boolean AlignRobotPB() {
        return gamepad1.y;
    }
    //public boolean IncrementOncePB(){return gamepad1.xWasReleased();}
    //public boolean IncrementOncePBV2(){return gamepad1.yWasReleased();}

    /****************************/
    // Emergency Stop Functions
    public void checkEmergencyStop() {
        if (emergencyButtons()) {
            stopDrive();
            //stop all motors, servos, etc.
            terminateOpModeNow();   // Force exit of OpMode
        }
    }


    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    // thumb stick values inside function body. This will allow tank drive to be reused for
    // autonomous programs without additional work
    public void tankdrive(double left, double right) {
        rightFront.setPower(right);
        rightBack.setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);

        telemetry.addData("right speed:", right);
        telemetry.addData("left speed", left);
    }

    public void stopDrive() {
        tankdrive(0, 0);
    }


    /*****************************/
    //shotgun Methods (Functions)
    public void shotgunSpiny(double speed) {
        shotgunMotor.setVelocity(speed);
    }

    public void moveTriggerServo(double position) {
        shotgunTriggerServo.setPosition(position);
    }

    public void gunTriggerSafety(double position) {
        if (SpindexInPosition()) {
            if (position == triggerUp) {
                telemetry.addData("shooting", "yes");
            } else {
                telemetry.addData("shooting", "no");
            }

            moveTriggerServo(position);
            didShooterShoot();
        }
    }

    // Matt we don't know if you want this but we tried - HJO & KB
    //This is to see if the shotgun shoots or not and updates the color array if needed/worked
    public boolean didShooterShoot() {
        if (ShotgunHasBall()) {
            return false;
        } else {
            colorArray[ShootgunIndex] = "None";
            return true;
        }
    }

    public void controlOfShotgun() {
        double shped = 1750; // 1630
        double NoTolerance = 100;
        double currentVelocity = shotgunMotor.getVelocity();

//        if (ShotgunHasBall()){
//            if (tempBulletColor == "None"){
//                tempBulletColor = colorArray[ShootgunIndex];
//            }else {
//                colorArray[ShootgunIndex] = tempBulletColor;
//            }
//        }

        if (shotgunSpinyPB) {
//            shotgunSpiny(shped);
            shotgunMotor.setVelocity(shped);
//            telemetry.addData("AAAAHHHHHHHHH","");
//            telemetry.update();

        } else {
            shotgunSpiny(0);
        }

        if (shotgunTriggerPB() && shped >= currentVelocity - NoTolerance && shped <= currentVelocity + NoTolerance) {
            gunTriggerSafety(triggerUp);
//            shotgunTriggerServo.setPosition(triggerUp); //No safety
            //Added didShooterShoot() to gunTriggerSafety()
        } else {
            //gunTriggerSafety(triggerDown);
            shotgunTriggerServo.setPosition(triggerDown);

        }
        if(shotgunTriggerWasReleased()){
            spindexLockoutDelayStart = runtime.milliseconds();
        }

    }


    public void shotgunSpeedTest() {
        double shped = shotgunSpeed; //1750 // 1630
        double NoTolerance = 100;
        double currentVelocity = shotgunMotor.getVelocity();

        if (IncrementorPlusButton()) {
            shotgunSpeed += 10;
        } else if (IncrementorMinusButton()) {
            shotgunSpeed -= 10;
        }

//        if (ShotgunHasBall()){
//            if (tempBulletColor == "None"){
//                tempBulletColor = colorArray[ShootgunIndex];
//            }else {
//                colorArray[ShootgunIndex] = tempBulletColor;
//            }
//        }

        if (shotgunSpinyPB) {
//            shotgunSpiny(shped);
            shotgunMotor.setVelocity(shped);
//            telemetry.addData("AAAAHHHHHHHHH","");
//            telemetry.update();

        } else {
            shotgunSpiny(0);
        }

        if (shotgunTriggerPB() && shped >= currentVelocity - NoTolerance && shped <= currentVelocity + NoTolerance) {
            //gunTriggerSafety(triggerUp);
            shotgunTriggerServo.setPosition(triggerUp);
            //Added didShooterShoot() to gunTriggerSafety()
        } else {
            //gunTriggerSafety(triggerDown);
            shotgunTriggerServo.setPosition(triggerDown);

        }
        if(shotgunTriggerWasReleased()){
            spindexLockoutDelayStart = runtime.milliseconds();
        }

    }


    /*****************************/
    //Autonomous Functions

    /****************************/
    //Motor Functions
    public void inputRubberMotor() {
        double shpeed = 1;
        if (rubberIntakePB()) {
            runRubberMotor(shpeed);
        } else if (rubberOuttakePB()) {
            runRubberMotor(-shpeed);
        } else {
            runRubberMotor(0);

        }
        // test if spindexer is not in posittion. it moves it
        /* else if (!SpindexInPosition()){
            runRubberMotor(shpeed/5);
        } */

    }

    public void inputRubberMotorExtreme() {
        double shpeed = 1;
        if (ToggleIntake) {
            runRubberMotorExtreme(shpeed);
        } else if (rubberOuttakePB()) {
            runRubberMotorExtreme(-shpeed);
        } else {
            runRubberMotorExtreme(0);
        }
        // test if spindexer is not in posittion. it moves it
        /* else if (!SpindexInPosition()){
            runRubberMotor(shpeed/5);
        } */

    }

    public void runRubberMotorExtreme(double shpeed) {
        if (shpeed < 0) {
            rubberIntake.setPower(shpeed);
            runBallKeeper(true, false);
        } else if (shpeed > 0) {
            rubberIntake.setPower(shpeed);
            runBallKeeper(false, false);
        }

/*        else if (SpindexInPosition() && shpeed > 0) {
            rubberIntake.setPower(shpeed);
            runBallKeeper(false, false);
        } else if (!SpindexInPosition() && shpeed > 0) {
            runBallKeeper(false, false);
        }
*/
        else {
            rubberIntake.setPower(0);
            runBallKeeper(false, true);
        }
    }

    public void runRubberMotor(double shpeed) {
        if (shpeed < 0) {
            rubberIntake.setPower(shpeed);
            runBallKeeper(true, false);
        } else if (SpindexInPosition() && shpeed > 0) {
            rubberIntake.setPower(shpeed);
            runBallKeeper(false, false);
        } else if (!SpindexInPosition() && shpeed > 0) {
            runBallKeeper(false, false);
        } else {
            rubberIntake.setPower(0);
            runBallKeeper(false, true);
        }
    }

    /*****************************/
    //Encoder Functions
   /*
    public double getLeftVerticalEncoder(){
        return rightFront.getCurrentPosition();
    }
    */

    /*****************************/
    //Gyro Functions

    /*****************************/
    //Servo Functions
    public void setServoPosition0(double ServoPos) {
        spindexServo.setPosition(ServoPos);
    }

    public void runBallKeeper(boolean backwards, boolean stop) {
        if (stop) {
            TheKeeperOfTheBalls.setPower(0);
        } else if (!backwards) {
            TheKeeperOfTheBalls.setPower(1);
        } else {
            TheKeeperOfTheBalls.setPower(-1);
        }
    }

    public void setSpindexerServo() {
        setServoPosition0(PosArray[spindexerArrayIndex]);
    }

    public boolean SpindexInPosition() {
        int whenSpinnyIsThereButNotFully = 10;
        int whereSpinnyCurrentlyIsAt = spinMotor.getCurrentPosition();
        int target = spinnyGoHere;
        //todo fix

        return ((whereSpinnyCurrentlyIsAt < target + whenSpinnyIsThereButNotFully) || (whereSpinnyCurrentlyIsAt > target - whenSpinnyIsThereButNotFully));

    }

    public boolean SpindexIsHome() {
        return !spindexPositionSwitch.getState();
    }

    public void SetSpindexPositionToShotgun(int spindexerIndex) {
        //TODO: This function
    }

    public void MoveSpindexServoV2() {
        if (!IsBall() && SpindexInPosition()) {
            if (spindex_left_was_Released()) {
                SpindexPosition = SpindexPosition - SpindexIncrement;
            }
        }
        if (spindex_right_was_released()) {
            SpindexPosition = SpindexPosition + SpindexIncrement;
        }
        if (SpindexPosition < 0) {
            SpindexPosition = 0.0128;
        }
        if (SpindexPosition > 1) {
            SpindexPosition = SpindexPosition - SpindexIncrement;
        }
        spindexServo.setPosition(SpindexPosition);
    }


    public void MoveSpindexServoV3() {
        if (colorPosIndex > 2) {
            colorPosIndex = colorPosIndex - 3;
        }

        if (shotgunTriggerServo.getPosition() == triggerDown) {
            if (!IsBall() && SpindexInPosition()) {
                if (spindex_left_was_Released()) {
                    hackyPosIndex -= 1;
                    colorPosIndex -= 1;
                }
            }
            if (spindex_right_was_released()) {
                hackyPosIndex += 1;
                colorPosIndex += 1;
            }

            if (hackyPosIndex < 0) {
                hackyPosIndex = 0;
                colorPosIndex = 0;
            }
            if (hackyPosIndex > 13) {
                colorPosIndex -= 1;
                hackyPosIndex -= 1;
            }

            if (spindexerResetPB() && rubberOuttakePB()) {
                hackyPosIndex = 0;
                colorPosIndex = 0;
                //todo: run outtake
            }

        }
        double spindexPos = hackyPosArray[hackyPosIndex];
        spindexServo.setPosition(spindexPos);
    }

    //can be made to function like minecraft hotbar, does not yet.
    public void MoveSpindexServo() {
        double currentPosition = spindexServo.getPosition();

        boolean moveLeft = false;
        boolean moveRight = false;

        if (spindex_left_was_Released()) {
            moveLeft = true;
        }

        if (spindex_right_was_released()) {
            moveRight = true;
        }

        if (moveRight || moveLeft) {

            if (spindexerArrayIndex == 0) {
                if (moveRight) {
                    spindexerArrayIndex += 1;
                }
            } else {
                if (spindexerArrayIndex == 1) {
                    if (moveRight) {
                        spindexerArrayIndex += 1;
                    } else {
                        spindexerArrayIndex -= 1;
                    }
                } else {
                    if (moveLeft) {
                        spindexerArrayIndex -= 1;
                    }
                }
            }
            moveRight = false;
            moveLeft = false;
        }
        if (spindexerArrayIndex == 0) {
            ShootgunIndex = 2;
        } else {
            ShootgunIndex = spindexerArrayIndex - 1;
        }
        //setSpindexerServo();
    }

    //new spindexer stuff
    public void spinnyHome() {
        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (SpindexIsHome()) {
            spinMotor.setPower(0);
            spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            spinMotor.setPower(0.1);
        }
    }

    public void MoveSpindexMotorV1() {
        if (colorPosIndex > 2) {
            colorPosIndex = colorPosIndex - 3;
        }

        if (shotgunTriggerServo.getPosition() == triggerDown) {
            if (!IsBall() && SpindexInPosition()) {
                if (spindex_left_was_Released()) {
                    spinnyGoHere -= SpindexMotorIncrement;
                    colorPosIndex -= 1;
                }
            }
            if (spindex_right_was_released()) {
                spinnyGoHere += SpindexMotorIncrement;
                colorPosIndex += 1;
            }

            if (spindexerResetPB() && rubberOuttakePB()) {
                spinnyGoHere = 0;
                colorPosIndex = 0;
                //todo: run outtake
            }

        }

        //Safety: Don't move unless enough time has passed for servo to go down
        if(runtime.milliseconds()>spindexLockoutDelayStart+spindexLockoutDelay){
            spinMotor.setTargetPosition(spinnyGoHere);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(0.25);
        }



        telemetry.addData("actual motor position", spinMotor.getCurrentPosition());
        telemetry.addData("motor tick target position", spinnyGoHere);
    }

    public boolean ColorGreen() {
        int MaxGreen = 195;
        int MinGreen = 80;

        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = bulletIntakeColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] >= MinGreen && hsvValues[0] <= MaxGreen) {
            return true;
        } else {
            return false;
        }
    }

    public String ColorIs() {
        String Color = "";
        if (IsBall()) {
            if (ColorGreen()) {
                Color = "Green";
            } else {
                Color = "Purple";
            }
        } else {
            Color = "None";
        }
        telemetry.addData("Color is", Color);
        return Color;
    }

    public boolean IsBall() {
        double DistanceSensing = 4;
        if
        (((DistanceSensor) bulletIntakeColorSensor).getDistance(DistanceUnit.CM) < DistanceSensing) {
            return true;
        } else {
            return false;
        }
    }

    public void setSpindexToZero() {
        hackyPosIndex = 0;

        double spindexPos = hackyPosArray[hackyPosIndex];
        spindexServo.setPosition(spindexPos);
    }

    public boolean ShotgunHasBall() {
        double DistanceSensing = 5;
        if
        (((DistanceSensor) shotgunLoadingColorSensor).getDistance(DistanceUnit.CM) < DistanceSensing) {
            return true;
        } else {
            return false;
        }
    }

    public void autoRead() {
        if (SpindexInPosition()) {
            ColorToPos();
        }
    }

    public void ColorToPos() {
        colorArray[spindexerArrayIndex] = ColorIs();
    }

    public String getbulletcolor() {
        int colorIndex;
        if (ShootgunIndex == 2) {
            colorIndex = 0;
        } else {
            colorIndex = ShootgunIndex + 1;
        }
        return colorArray[colorIndex];
    }

    public void colorTelemetry() {
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = bulletIntakeColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) bulletIntakeColorSensor).getDistance(DistanceUnit.CM));
//
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
    }

    public void displayTelemetry() {
        telemetry.addData("Spindex in position", SpindexInPosition());
        telemetry.addData("Spindex INDEX!", hackyPosIndex);
        telemetry.addData("Flywheel Button:", shotgunSpinyPB);
        telemetry.addData("Snail is", snailDirection);

        ColorIs();

//        telemetry.addData("spindex servo position", getSpindexPosition());
////        telemetry.addData("L pressed",gamepad1.leftBumperWasPressed());
////        telemetry.addData("R pressed",gamepad1.rightBumperWasPressed());
//        telemetry.addData("L released",gamepad1.leftBumperWasReleased());
//        telemetry.addData("R released",gamepad1.rightBumperWasReleased());
        telemetry.addData("shotgun target", shotgunSpeed);
        telemetry.addData("shotgun velocity", shotgunMotor.getVelocity());
//        telemetry.addData("shotgun has ball: ", ShotgunHasBall());
//        for (int i = 0; i<3; i++){
//            telemetry.addData("color in index " + i, colorArray[i]);
//        }
//        telemetry.addData("spindex array index", spindexerArrayIndex);
//        telemetry.addData("Shotgun Index",ShootgunIndex);
//        telemetry.addData("color in Shotgun ",getbulletcolor());

        colorTelemetry();

    }

    public double getAngler() {
        double angle;

        pos = OdometrySensor.getPosition();


        angle = pos.h;
        return angle;
    }

    public void holonomicFieldCentric() {
        double rotateVelocity = right_sticky_x();

        double offset = Math.toRadians(90);
        double robotHead = getAngler();
        double LeftStickValuex = left_sticky_x();
        double LeftStickValueY = left_sticky_y();
        if (rubberToggleIntakePB()) {
            ToggleIntake = !ToggleIntake;
        }
        if (shotgunToggleSpinyPB()) {
            shotgunSpinyPB = !shotgunSpinyPB;
        }
        if (slowDownStick()) {
            ToggleSpeed = !ToggleSpeed;
        }

        if (ToggleSpeed) {
            double SlowDownStick = 0.5;
            rotateVelocity = rotateVelocity * SlowDownStick;
            LeftStickValuex = LeftStickValuex * SlowDownStick;
            LeftStickValueY = LeftStickValueY * SlowDownStick;
        }

        double angle = Math.atan2(left_sticky_y(), left_sticky_x()) - Math.toRadians(robotHead) - offset;
        double magnitude = Math.hypot(LeftStickValuex, LeftStickValueY);

        double Vlf = (magnitude * Math.cos(angle + (Math.PI / 4)) + rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle + (Math.PI / 4)) + rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle + (Math.PI / 4)) - rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle + (Math.PI / 4)) - rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        if (FieldCentricReset()) {
            ZeroOtosSensor();
        }

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);


//        telemetry.addData("robot heading", robotHead);
        telemetry.addData("robot heading", robotHead);
        telemetry.addData("drive angle", Math.toDegrees(angle));
    }

    public void autoHolonomicFieldCentric(double magnitude, double angle, double rotateVelocity) {
        double robotHead = getAngler();
//        double offset = Math.toRadians(-90+robotHead);
//        angle = Math.toRadians(angle)+offset;
        double offset = Math.toRadians(90); //-90
        angle = angle - Math.toRadians(robotHead) - offset;

        double Vlf = (magnitude * Math.cos(angle + (Math.PI / 4)) + rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle + (Math.PI / 4)) + rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle + (Math.PI / 4)) - rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle + (Math.PI / 4)) - rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);
        telemetry.addData("heading: ", robotHead);
        telemetry.addData("drive direction: ", angle);
    }


    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        OdometrySensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        OdometrySensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, -90);
        OdometrySensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        OdometrySensor.setLinearScalar(117.75 / 99.7468);
        OdometrySensor.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        OdometrySensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        OdometrySensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        OdometrySensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        OdometrySensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    public void ZeroOtosSensor() {
        OdometrySensor.resetTracking();
    }

    public void holonomic() {
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());
        double offset = Math.toRadians(-90);
        double angle = Math.atan2(left_sticky_y(), left_sticky_x()) + offset;
        double rotateVelocity = right_sticky_x();

        double Vlf = (magnitude * Math.cos(angle + (Math.PI / 4)) + rotateVelocity);
        double Vlb = (magnitude * Math.sin(angle + (Math.PI / 4)) + rotateVelocity);
        double Vrf = (magnitude * Math.sin(angle + (Math.PI / 4)) - rotateVelocity);
        double Vrb = (magnitude * Math.cos(angle + (Math.PI / 4)) - rotateVelocity);

        leftFront.setPower(Vlf);
        leftBack.setPower(Vlb);
        rightFront.setPower(Vrf);
        rightBack.setPower(Vrb);

        telemetry.addData("lf", Vlf);
        telemetry.addData("lb", Vlb);
        telemetry.addData("rf", Vrf);
        telemetry.addData("rb", Vrb);
    }

    public void toggleSnail() {
        if (SnailPB_was_Released()) {
            RunSnail();
        }
    }

    public void RunSnail() {
        double morePower = 0;
        double snailPos = snailServo.getPosition();

        if (snailPos == 0) {
            morePower = snailPosition;
            snailDirection = "extended";
        } else {
            morePower = 0;
            snailDirection = "retracted";
        }
        snailServo.setPosition(morePower);
    }

    public double getSpindexPosition() {
        return spindexServo.getPosition();
    }


    public void MoveServo(boolean down) {
        double movement = spindexServo.getPosition();
        if (down) {
            movement = movement - 0.0005;
        } else {
            movement = movement + 0.0005;
        }
        if (movement > 1) {
            movement = 1;
        } else if (movement < 0) {
            movement = 0;
        }

        telemetry.addData("Servo 5 Position: ", movement);
        spindexServo.setPosition(movement);
    }

    public void RunTesting() {
//        TestingSmallIncrementV2();
//        TestingSmallIncrementV2();
        if (IncrementorPlusButton()) {
            MoveServo(false);
        } else if (IncrementorMinusButton()) {
            MoveServo(true);
        }
    }

//    public void prerecordColors(){
//        setServoPosition0(servoPosition0);
//        while (!SpindexInPosition()){
//        }
//        autoRead();
//        setServoPosition0(servoPosition1);
//        while (!SpindexInPosition()){
//        }
//        autoRead();
//        setServoPosition0(servoPosition2);
//        while (!SpindexInPosition()){
//        }
//        autoRead();
//        setServoPosition0(servoPosition0);
//    }
//

    public void prerecordColors() {
        setServoPosition0(servoPosition0);
        while (!SpindexInPosition()) {
        }
        autoRead();
        setServoPosition0(servoPosition1);
        while (!SpindexInPosition()) {
        }
        autoRead();
        setServoPosition0(servoPosition2);
        while (!SpindexInPosition()) {
        }
        autoRead();
        setServoPosition0(servoPosition0);
    }

    public void MoveSpindexMotorautomosv1() {
        spinnyGoHere += SpindexMotorIncrement;
        colorPosIndex += 1;

        if (colorPosIndex > 2) {
            colorPosIndex = colorPosIndex - 3;
        }
        spinMotor.setTargetPosition(spinnyGoHere);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(0.25);

        telemetry.addData("actual motor position", spinMotor.getCurrentPosition());
        telemetry.addData("motor tick target position", spinnyGoHere);
    }
    public void Movehoodshootv1(int ShootHowMany) {
        shotgunSpiny(shotgunSpeed);
        sleep(2000);
        for(int i=0;i<ShootHowMany;i++){
            moveTriggerServo(triggerUp);
            sleep(1500);
            moveTriggerServo(triggerDown);
            sleep(1500);
            MoveSpindexMotorautomosv1();
            sleep(1500);
        }
        shotgunSpiny(0);
    }





/*
    public void TestingSmallIncrementV2(){
        if (IncrementOncePBV2()){
            MoveServo(true);
        }
    }
    public void TestingSmallIncrement(){
        if (IncrementOncePB()){
            MoveServo(false);
        }
    }
*/
}
/*
TODO:   not listed in any particular order of importance..................................
    ......................................................................................
    ......................................................................................
    Auto index on intake
    .......................................................................................
    .......................................................................................
    Shooter - Trigger Servo functions (auto and manual) - should only need 2 positions
    Shooter - velocity control (needs DcMotorEx...)
    Intake - intake motor-
    Intake/Spindexer - record ball color in color array X-
    Intake/Spindexer - rotate after ball detected
    Intake/Spindexer - Run intake while cycling spindexer to keep ball in?-
    Intake/Spindexer - Reverse intake to eject extra ball
    Spindexer initialization (auto) - Cycle through positions, record ball colors
    Spindexer - Button to select/cycle to color
    Shooter/Spindexer - change color to "empty" or similar after successful launch
    Color Sensor - add sensors for pickup and launch locations.
        - Launch position used for detection only (distance), no color
        - Integrate Color sensor code into ParentOpMode-
    .......................................................................................
    .......................................................................................
*/