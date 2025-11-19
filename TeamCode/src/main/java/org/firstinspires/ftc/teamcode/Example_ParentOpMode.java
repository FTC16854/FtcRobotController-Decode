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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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
public class Example_ParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();
    private NormalizedColorSensor colorSensor;

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor shotgunMotor = null;
    private DcMotor rubberIntake = null;

    private Servo shotgunTriggerServo = null;
    private Servo spindexServo = null;

    private DigitalChannel spindexPositionSwitch = null;
    //Other Global Variables
    //put global variables here...
    double servoPosition0 = 0;
    double servoPosition1 = 0.33;
    double servoPosition2 = 0.67;

    String[] colorArray = new String[3];
    double[] PosArray = {servoPosition0, servoPosition1, servoPosition2};
    int spindexerArrayIndex = 0;

    final float colorSensorGain = (float) 17.5;


    //TODO: Create array for ball colors (uses same index as array for position)

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get()' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).

        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftFront = hardwareMap.get(DcMotor.class,"lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        shotgunMotor = hardwareMap.get(DcMotor.class,"shooter");
        rubberIntake = hardwareMap.get(DcMotor.class,"intake");



        shotgunTriggerServo = hardwareMap.get(Servo.class, "triggerServo");
        spindexServo = hardwareMap.get(Servo.class, "spindex Servo");

        spindexPositionSwitch = hardwareMap.get(DigitalChannel.class,"spindex position switch");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(colorSensorGain);
        //Set motor run mode (esp. if using SPARK Mini motor controller(s) for drivetrain)

        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        shotgunMotor.setDirection(DcMotor.Direction.FORWARD);
        rubberIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        spindexServo.setDirection(Servo.Direction.FORWARD);
        shotgunTriggerServo.setDirection(Servo.Direction.FORWARD);

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shotgunMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rubberIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        while(opModeInInit()){
            telemetry.addData("Cheese and Pickles ","Sandwich!");
            // Code in here will loop continuously until OpMode is started
        }

        // Wait for the game to start (driver presses PLAY) - May not be needed if using an init Loop
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes


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
    public double left_sticky_x(){return gamepad1.left_stick_x;}
    public double left_sticky_y() { return -gamepad1.left_stick_y;}
    public double right_sticky_y() { return -gamepad1.right_stick_y;}
    public double right_sticky_x() { return  gamepad1.right_stick_x;}
    

    // Buttons
    public boolean spindex_left(){return gamepad1.left_bumper;}
    public boolean spindex_left_was_pressed(){return gamepad1.leftBumperWasPressed();}
    public boolean spindex_left_was_Released(){return gamepad1.leftBumperWasReleased();}

    public boolean spindex_right(){return gamepad1.right_bumper;}
    public boolean spindex_right_was_pressed(){return gamepad1.rightBumperWasPressed();}
    public boolean spindex_right_was_released(){return gamepad1.rightBumperWasReleased();}

    public boolean shotgunspiny (){return gamepad1.a;}

    public boolean shotgunTrigger(){return gamepad1.right_trigger >0.5;}

    public boolean rubberIntake(){return gamepad2.left_bumper;}
    public boolean rubberOuttake(){return gamepad2.right_bumper;}

    public boolean emergencyButtons(){
        // check for combination of buttons to be pressed before returning true
        return (gamepad1.b && gamepad1.y) || (gamepad2.b && gamepad2.y);
    }


    /****************************/
    // Emergency Stop Functions
    public void checkEmergencyStop(){
        if(emergencyButtons()){
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
    public void tankdrive(double left, double right){
        rightFront.setPower(right);
        rightBack. setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);

        telemetry.addData("right speed:", right);
        telemetry.addData("left speed", left);
    }

    public void stopDrive (){tankdrive(0, 0);}




    /*****************************/
    //shotgun Methods (Functions)

    public void shotgunSpiny(double speed){shotgunMotor.setPower(speed);}
    public void moveTriggerServo(double position){shotgunTriggerServo.setPosition(position);}

    public void gunTriggeringSafety(double position){
        if(SpindexInPosition()){
            moveTriggerServo(position);
        }
    }
    /*****************************/
    //Autonomous Functions

    /****************************/
    //Motor Functions
    public void inputRubberMotor() {
        double shpeed = 0.5;

        if (rubberIntake()) {
                runRubberMotor(shpeed);
        } else if (rubberOuttake()) {
            runRubberMotor(-shpeed);
        } else{
            runRubberMotor(0);
        }
    }

    public void runRubberMotor(double shpeed){
        if(shpeed<0) {
            rubberIntake.setPower(shpeed);
        } else if (SpindexInPosition()) {
            rubberIntake.setPower(shpeed);
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
    public void setServoPosition0(double ServoPos){spindexServo.setPosition(ServoPos);}

    public void setSpindexerServo(){setServoPosition0(PosArray[spindexerArrayIndex]);}

    public boolean SpindexInPosition(){return  spindexPositionSwitch.getState();}
    //can be made to function like minecraft hotbar, does not yet.
    public void MoveServo(){
        double currentPosition = spindexServo.getPosition();

        boolean moveLeft = false;
        boolean moveRight = false;

        if(spindex_left_was_Released() || spindex_left_was_pressed()){
            moveLeft = true;
        }

        if(spindex_right_was_released() || spindex_right_was_pressed()){
            moveRight = true;
        }

        if (moveRight || moveLeft){

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
        setSpindexerServo();
    }

    public boolean ColorGreen() {
        int MaxGreen = 195;
        int MinGreen = 80;

        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if (hsvValues[0] >= MinGreen && hsvValues[0] <= MaxGreen) {
            return true;
        } else {
            return false;
        }
    }

    public void ColorIs() {
        if (IsBall()) {
            if (ColorGreen()) {
                telemetry.addData("Color: ", "green");
            } else {
                telemetry.addData("Color: ", "purple");
            }
        } else {
            telemetry.addData("Color: ", "none");
        }
    }

    public boolean IsBall() {
        double DistanceSensing = 5;
        if
        (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) < DistanceSensing) {
            return true;
        } else {
            return false;
        }
    }

    public void colorTelemetry(){
        final float[] hsvValues = new float[3];
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
    }

    public void telemetry(){
        telemetry.addData("spindex array index", spindexerArrayIndex);
        telemetry.addData("spindex servo position", spindexServo.getPosition());
        telemetry.addData("L pressed",gamepad1.leftBumperWasPressed());
        telemetry.addData("R pressed",gamepad1.rightBumperWasPressed());
        telemetry.addData("L released",gamepad1.leftBumperWasReleased());
        telemetry.addData("R released",gamepad1.rightBumperWasReleased());
        colorTelemetry();
        ColorIs();
    }
}

/*
TODO:   not listed in any particular order of importance...
    .......................................................................................
    .......................................................................................
    Shooter - Trigger Servo functions (auto and manual) - should only need 2 positions
    Shooter - velocity control (needs DcMotorEx...)
    Intake - intake motor
    Intake/Spindexer - record ball color in color array
    Intake/Spindexer - rotate after ball detected
    Intake/Spindexer - Run intake while cycling spindexer to keep ball in?
    Spindexer - Fix Edge-detection (wasPressed/WasReleased)
    Spindexer Positions - Limit Switch(es) for pickup/shoot position
    Spindexer initialization (auto) - Cycle through positions, record ball colors
    Spindexer - Button to select/cycle to color
    Shooter/Spindexer - change color to "empty" or similar after successful launch
    Color Sensor - add sensors for pickup and launch locations.
        - Launch position used for detection only (distance), no color
        - Integrate Color sensor code into ParentOpMode
            - Color sensor for intake
            - Color (Distance) sensor for shooter
    .......................................................................................
    .......................................................................................
*/