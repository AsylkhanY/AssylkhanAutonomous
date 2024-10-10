package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RedAutonomous")

public class RedAutonomous extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    public OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels



    //APRIL TAG VARIABLES


    //OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;

    //ODOMETER VARIABLES

    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front_drive", rbName = "right_back_drive", lfName = "left_front_drive", lbName = "left_back_drive";
    DcMotor intake;
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = lbName, horizontalEncoderName = rfName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    //MY VARIABLES

    boolean GetToBackDrop = false;

    double initialX , initialY;
    int NeededID;
    //private DcMotor Lift1 = null;
    //private DcMotor Lift2 = null;
    //private Servo leftServo = null;
    //private Servo rightServo = null;
    //private ColorSensor colorsensor = null;

    double ticks = 560;
    double newTarget;
    boolean center,right,left;


    double x = 400;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private Servo box = null;
    private Servo holder = null;
    private CRServo cylinder = null;

    private Servo leftBox = null;
    private Servo rightBox = null;
    private Servo droneHolder = null;
    private Servo drone = null;


    @Override
    public void runOpMode() {

        initOpenCV();
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftLift = hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = hardwareMap.get(DcMotor.class, "right_lift");
        box = hardwareMap.get(Servo.class, "box");
        holder = hardwareMap.get(Servo.class, "holder");
        cylinder = hardwareMap.get(CRServo.class, "cylinder");
        leftBox = hardwareMap.get(Servo.class, "left_box");
        rightBox = hardwareMap.get(Servo.class, "right_box");
        droneHolder = hardwareMap.get(Servo.class, "drone_holder");
        drone = hardwareMap.get(Servo.class, "drone");

        leftLift.setDirection(DcMotor.Direction.REVERSE);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        waitForStart();
        initialX = cX;
        initialY = cY;

        telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
        telemetry.addData("Distance in cm", (getDistance(width)));
        telemetry.update();
        drone.setPosition(1);
        holder.setPosition(0.5);

        box.setPosition(0.38);
        droneHolder.setPosition(1);
        goToPosition(10 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, -90, 7 * COUNTS_PER_INCH, 5,true);

        while (opModeIsActive()) {
            sleep(500);
            if(initialX > 520 || gamepad1.b){
                NeededID = 6;
                right = true;
            }
            else if((initialX > 280 && initialX < 520) || gamepad1.a){
                NeededID = 5;
                center = true;
            }
            else{
                NeededID = 4;
                left = true;
            }
            controlHubCam.stopStreaming();

            if(left){
                //sleep(10000);
                goToPosition(10 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, -90, 7 * COUNTS_PER_INCH, 5,true);
                goToPosition(-3 * COUNTS_PER_INCH, 30 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);
                cylinder.setPower(1);
                sleep(2000);
                cylinder.setPower(0);
                goToPosition(25 * COUNTS_PER_INCH, 35.3 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5,true);
                goToPosition(39 * COUNTS_PER_INCH, 35.3 * COUNTS_PER_INCH, 0.5, -90, 1 * COUNTS_PER_INCH, 1.5,false);
                leftBox.setPosition(0.25);
                rightBox.setPosition (0.75);
                sleep(1300);
                left_front.setPower(-0.2);
                left_back.setPower(-0.2);
                right_front.setPower(-0.2);
                right_back.setPower(-0.2);
                sleep(500);
                holder.setPosition(0.25);
                left_front.setPower(0);
                left_back.setPower(0);
                right_front.setPower(0);
                right_back.setPower(0);
                sleep(500);
                leftBox.setPosition(1);
                rightBox.setPosition(0);
                holder.setPosition(1);
                goToPosition(29 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,true);
                goToPosition(45 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);
            }
            else if(center){
                //sleep(10000);
                goToPosition(10 * COUNTS_PER_INCH, 39 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);
                //goToPosition(0 * COUNTS_PER_INCH, 32 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);

                cylinder.setPower(1);
                sleep(2000);
                cylinder.setPower(0);
                goToPosition(25 * COUNTS_PER_INCH, 29.5 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5,true);
                goToPosition(39 * COUNTS_PER_INCH, 29.5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 1.5,false);
                leftBox.setPosition(0.25);
                rightBox.setPosition (0.75);
                sleep(1300);
                left_front.setPower(-0.2);
                left_back.setPower(-0.2);
                right_front.setPower(-0.2);
                right_back.setPower(-0.2);
                sleep(500);
                holder.setPosition(0.25);
                left_front.setPower(0);
                left_back.setPower(0);
                right_front.setPower(0);
                right_back.setPower(0);
                sleep(500);
                leftBox.setPosition(1);
                rightBox.setPosition(0);
                holder.setPosition(1);
                goToPosition(29 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,true);
                goToPosition(45 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);

            }
            else{
                //sleep(10000);
                goToPosition(18.5 * COUNTS_PER_INCH, 35 * COUNTS_PER_INCH, 0.5, -90, 1.5 * COUNTS_PER_INCH, 2,false);
                //goToPosition(11.5 * COUNTS_PER_INCH, 22 * COUNTS_PER_INCH, 0.5, 0, 1.5 * COUNTS_PER_INCH, 2,false);
                cylinder.setPower(1);
                sleep(2000);
                cylinder.setPower(0);
                goToPosition(30 * COUNTS_PER_INCH, 25.5 * COUNTS_PER_INCH, 0.5, -90, 5 * COUNTS_PER_INCH, 5,true);
                goToPosition(39 * COUNTS_PER_INCH, 25.5 * COUNTS_PER_INCH, 0.5, -90, 1 * COUNTS_PER_INCH, 1.5,false);
                leftBox.setPosition(0.25);
                rightBox.setPosition (0.75);
                sleep(1300);
                left_front.setPower(-0.2);
                left_back.setPower(-0.2);
                right_front.setPower(-0.2);
                right_back.setPower(-0.2);
                sleep(500);
                holder.setPosition(0.25);
                left_front.setPower(0);
                left_back.setPower(0);
                right_front.setPower(0);
                right_back.setPower(0);
                sleep(500);
                leftBox.setPosition(1);
                rightBox.setPosition(0);
                holder.setPosition(1);
                goToPosition(29 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,true);
                goToPosition(47 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0.5, -90, 2 * COUNTS_PER_INCH, 2,false);

            }
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            globalPositionUpdate.stop();
            sleep(10000);
            break;

            //AprilTag(NeededID);
            /*
            while(x <= 280 || x >= 520){
                if(x < 280){
                }
                else if(x > 520){
                }
            }
            */
        }

        globalPositionUpdate.stop();
        // Release resources
        controlHubCam.stopStreaming();
    }


    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableError,double allowableErrorDegrees, boolean passThrough){
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();


        double distance =  Math.hypot(distanceToXTarget,distanceToYTarget);

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

            double decelerationDistance = 4 * COUNTS_PER_INCH;
            double minSpeed = 0.25;
            //double deceleration = Math.max(minSpeed * decelerationDistance, Math.min(decelerationDistance, distance)) / decelerationDistance;
            //double deceleration = Math.max(minSpeed, Math.min(distance / initialDistance, 1));
            double deceleration = 1;
            if(distance < decelerationDistance){
                deceleration = 0.15;
            }

            if(distance > allowableError) {
                leftFrontSpeed = ((cosComponent - sinComponent) * robot_movement_y_component) + ((sinComponent + cosComponent) * robot_movement_x_component * 1.3) * deceleration;
                leftBackSpeed = ((cosComponent + sinComponent) * robot_movement_y_component) + ((sinComponent - cosComponent) * robot_movement_x_component * 1.3) * deceleration;
                rightFrontSpeed = ((cosComponent + sinComponent) * robot_movement_y_component) + ((sinComponent - cosComponent) * robot_movement_x_component * 1.3) * deceleration;
                rightBackSpeed = ((cosComponent - sinComponent) * robot_movement_y_component) + ((sinComponent + cosComponent) * robot_movement_x_component * 1.3) * deceleration;
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

            if(distance < allowableError && Math.abs(pivotCorrection) < allowableErrorDegrees && !passThrough){
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
        if(!passThrough) {
            left_front.setPower(0);
            left_back.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);
            right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        int sleepTime = 0;
        while(sleepTime < 500 && !passThrough){
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarget,distanceToYTarget);
            pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            if(Math.abs(distance) > allowableError || Math.abs(pivotCorrection) > allowableErrorDegrees){
                goToPosition(targetXPosition,targetYPosition,robotPower * 0.98,desiredRobotOrientation,allowableError,allowableErrorDegrees,passThrough);
            }
            sleep(10);
            sleepTime += 10;
        }
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

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        //controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
    }
    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);
            //255 0 0
            //0 0 255


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }

    public void AprilTag(int NeededID)
    {
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        controlHubCam.setPipeline(aprilTagDetectionPipeline);
        //controlHubCam.stopStreaming();
        //controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
        /*
        controlHubCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //controlHubCam.stopStreaming();
                controlHubCam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

         */

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!GetToBackDrop)
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == NeededID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    x = tagOfInterest.pose.x;
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null)
        {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        }
        else
        {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if(tagOfInterest.pose.x <= 20)
            {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50)
            {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50)
            {
                // do something else
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}

