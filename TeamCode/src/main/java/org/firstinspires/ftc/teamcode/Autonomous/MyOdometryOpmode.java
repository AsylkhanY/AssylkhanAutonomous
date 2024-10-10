package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Autonomous.OdometryGlobalCoordinatePosition;

import java.io.File;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    DcMotor intake;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo box = null;
    private Servo holder = null;

    private CRServo cylinder = null;

    private Servo leftBox = null;
    private Servo rightBox = null;

    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front_drive", rbName = "right_back_drive", lfName = "left_front_drive", lbName = "left_back_drive";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = lbName, horizontalEncoderName = rfName;


    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(14.89));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(-2000));
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        //rightLift = hardwareMap.get(DcMotor.class, "right_lift");
        //cylinder = hardwareMap.get(CRServo.class, "cylinder");
        //leftBox = hardwareMap.get(Servo.class, "left_box");
        //rightBox = hardwareMap.get(Servo.class, "right_box");

        //box = hardwareMap.get(Servo.class, "box");
        //holder = hardwareMap.get(Servo.class, "holder");
        //pixelHolder = hardwareMap.get(Servo.class, "pixelHolder");
        //leftLift.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();
        //pixelHolder.setPosition(1);

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        while(opModeIsActive()){
            //box.setPosition(0.38);
            sleep(100);

            //RED CLOSE LEFT
            /*
            goToPosition(10 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, -90, 7 * COUNTS_PER_INCH, 5);
            goToPosition(-5 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(25 * COUNTS_PER_INCH, 39 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(40 * COUNTS_PER_INCH, 39 * COUNTS_PER_INCH, 0.4, -90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(35 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 1);

            */


            //RED CLOSE CENTER
            /*
            goToPosition(10 * COUNTS_PER_INCH, 40 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(25 * COUNTS_PER_INCH, 29 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(40 * COUNTS_PER_INCH, 29 * COUNTS_PER_INCH, 0.4, -90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(35 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 1);


             */


            //RED CLOSE RIGHT
            /*
            goToPosition(20 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, -90, 1.5 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(30 * COUNTS_PER_INCH, 22.5 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(41.5 * COUNTS_PER_INCH, 23 * COUNTS_PER_INCH, 0.4, -90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(35 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 1);


             */


            //BLUE CLOSE CENTER
            /*
            goToPosition(-10 * COUNTS_PER_INCH, 43 * COUNTS_PER_INCH, 0.5, 90, 2 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(-25 * COUNTS_PER_INCH, 28.5 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(-43 * COUNTS_PER_INCH, 29 * COUNTS_PER_INCH, 0.4, 90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(-35 * COUNTS_PER_INCH, 1 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);


             */



            //BLUE CLOSE LEFT
            /*
            goToPosition(-20 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, 90, 1.5 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(-30 * COUNTS_PER_INCH, 22.5 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(-41.5 * COUNTS_PER_INCH, 23 * COUNTS_PER_INCH, 0.4, 90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(-35 * COUNTS_PER_INCH, 1 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);

             */

            //BLUE CLOSE RIGHT
            /*
            goToPosition(-10 * COUNTS_PER_INCH, 32 * COUNTS_PER_INCH, 0.5, 90, 7 * COUNTS_PER_INCH, 5);
            goToPosition(4 * COUNTS_PER_INCH, 32 * COUNTS_PER_INCH, 0.5, 90, 2 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(-30 * COUNTS_PER_INCH, 38 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(-42 * COUNTS_PER_INCH, 38 * COUNTS_PER_INCH, 0.4, 90, 2 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(350);
            leftLift.setPower(0);
            rightLift.setPower(0);
            sleep(1400);
            holder.setPosition(0);
            goToPosition(-35 * COUNTS_PER_INCH, 1 * COUNTS_PER_INCH, 0.5, 90, 5 * COUNTS_PER_INCH, 5);

             */

            /*
            goToPosition(-20 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.8, 90, 1.5 * COUNTS_PER_INCH, 2);
            cylinder.setPower(1);
            sleep(2000);
            cylinder.setPower(0);
            goToPosition(-30 * COUNTS_PER_INCH, 21 * COUNTS_PER_INCH, 0.8, 90, 5 * COUNTS_PER_INCH, 5);
            goToPosition(-39 * COUNTS_PER_INCH, 21.5 * COUNTS_PER_INCH, 0.7, 90, 1 * COUNTS_PER_INCH, 1);
            leftLift.setPower(0.8);
            rightLift.setPower(0.8);
            sleep(500);
            leftLift.setPower(0);
            rightLift.setPower(0);
            leftLift.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
            leftBox.setPosition(1);
            rightBox.setPosition(0);
            sleep(1800);
            holder.setPosition(0);
            goToPosition(-15 * COUNTS_PER_INCH, 60 * COUNTS_PER_INCH, 0.8, 90, 5 * COUNTS_PER_INCH, 5);
            holder.setPosition(0);
            leftLift.setPower(-0.8);
            rightLift.setPower(-0.8);
            sleep(500);
            leftLift.setPower(0);
            rightLift.setPower(0);
            leftLift.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE);
            leftBox.setPosition(0);
            rightBox.setPosition(1);
            sleep(1800);

            goToPosition(50 * COUNTS_PER_INCH, 60 * COUNTS_PER_INCH, 0.8, 90, 5 * COUNTS_PER_INCH, 5);

             */

            /*
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            globalPositionUpdate.stop();
            sleep(100000);

             */
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Telemetry();

        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void Telemetry(){
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

        //telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.update();
    }


    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableError,double allowableErrorDegrees){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance =  Math.hypot(distanceToXTarget,distanceToYTarget);
        double initialDistance = distance;

        //double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
        double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

        while(opModeIsActive() && (distance > allowableError || Math.abs(pivotCorrection) > allowableErrorDegrees)){
            double leftFrontSpeed = 0,leftBackSpeed = 0, rightFrontSpeed = 0,rightBackSpeed = 0;

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance =  Math.hypot(distanceToXTarget,distanceToYTarget);
            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

            double robot_movement_x_component = calculateX(robotMovementAngle, robotPower);
            double robot_movement_y_component = calculateY(robotMovementAngle, robotPower);
            double sinComponent = Math.sin(Math.toRadians(globalPositionUpdate.returnOrientation()));
            double cosComponent = Math.cos(Math.toRadians(globalPositionUpdate.returnOrientation()));

            pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double decelerationDistance = 5 * COUNTS_PER_INCH;
            double minSpeed = 0.25;
            //double deceleration = Math.max(minSpeed * decelerationDistance, Math.min(decelerationDistance, distance)) / decelerationDistance;
            //double deceleration = Math.max(minSpeed, Math.min(distance / initialDistance, 1));
            double deceleration = 1;
            if(distance < decelerationDistance){
                deceleration = 0.5;
            }

            if(distance > allowableError) {
                leftFrontSpeed = ((cosComponent - sinComponent) * robot_movement_y_component) + ((sinComponent + cosComponent) * robot_movement_x_component * 1.5) * deceleration;
                leftBackSpeed = ((cosComponent + sinComponent) * robot_movement_y_component) + ((sinComponent - cosComponent) * robot_movement_x_component * 1.5) * deceleration;
                rightFrontSpeed = ((cosComponent + sinComponent) * robot_movement_y_component) + ((sinComponent - cosComponent) * robot_movement_x_component * 1.5) * deceleration;
                rightBackSpeed = ((cosComponent - sinComponent) * robot_movement_y_component) + ((sinComponent + cosComponent) * robot_movement_x_component * 1.5) * deceleration;
            }
            /*
            if(Math.abs(distanceToYTarget) > allowableError){
                double val = (distanceToYTarget / Math.abs(distanceToYTarget));
                leftFrontSpeed += robot_movement_y_component;
                leftBackSpeed += robot_movement_y_component;
                rightFrontSpeed += robot_movement_y_component;
                rightBackSpeed += robot_movement_y_component;
            }
            if(Math.abs(distanceToXTarget) > allowableError){
                double val = (distanceToXTarget / Math.abs(distanceToXTarget));
                leftFrontSpeed += robot_movement_x_component * 1.8;
                leftBackSpeed -= robot_movement_x_component * 1.8;
                rightFrontSpeed -= robot_movement_x_component * 1.8;
                rightBackSpeed += robot_movement_x_component * 1.8;
            }
             */
            if(Math.abs(pivotCorrection) > allowableErrorDegrees){
                double val = (pivotCorrection / Math.abs(pivotCorrection));
                leftFrontSpeed += val * robotPower * 0.5;
                leftBackSpeed += val * robotPower * 0.5;
                rightFrontSpeed -= val * robotPower * 0.5;
                rightBackSpeed -= val * robotPower * 0.5;
            }

            if(distance < allowableError && Math.abs(pivotCorrection) < allowableErrorDegrees){
                left_front.setPower(0);
                left_back.setPower(0);
                right_front.setPower(0);
                right_back.setPower(0);
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            else {
                left_front.setPower(leftFrontSpeed * 0.7);
                left_back.setPower(leftBackSpeed);
                right_front.setPower(rightFrontSpeed * 0.7);
                right_back.setPower(rightBackSpeed);
            }


            //distance =  Math.hypot(distanceToXTarget,distanceToYTarget);
            telemetry.addData("pivotcorrection", pivotCorrection);
            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
            Telemetry();
        }
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int sleepTime = 0;
        while(sleepTime < 500){
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            if(Math.abs(distance) > allowableError || Math.abs(pivotCorrection) > allowableErrorDegrees){
                goToPosition(targetXPosition,targetYPosition,robotPower * 0.98,desiredRobotOrientation,allowableError,allowableErrorDegrees);
            }
            sleep(10);
            sleepTime += 10;
        }
    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
