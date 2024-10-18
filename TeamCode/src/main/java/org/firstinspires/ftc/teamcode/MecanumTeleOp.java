package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("TopLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BottomLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("TopRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BottomRightMotor");
        DcMotor testMotor = hardwareMap.dcMotor.get("ArmMotor");

//        double integralSum = 0;
//        double Kp = 0;
//        double Ki = 0;
//        double Kd = 0;
//
        ElapsedTime timer = new ElapsedTime();
        double lastError = 0;
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double xs = Math.pow(gamepad1.left_stick_x, 2);
            double ys = Math.pow(gamepad1.left_stick_y, 2);
            double speed = Math.sqrt(xs + ys);
            telemetry.addData("Speed/radius is", String.format("%s", speed));
            telemetry.update();
            if (speed < 0.7) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } else {
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                DcMotor armLeft = hardwareMap.dcMotor.get("armLeft");
                DcMotor armRight = hardwareMap.dcMotor.get("armRight");
                armRight = hardwareMap.dcMotor.get("armRight");

                // Reverse backwards arm motor
                armRight.setDirection(DcMotor.Direction.REVERSE);

                // Set arm encoders to 0
                armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Set arm run mode
                armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Zero Power Behavior
                armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                // Setup Telemetry, will not clear after cycle, setup reusable items for output
                telemetry.setAutoClear(false);
                Telemetry.Item armLeftPosition = telemetry.addData("Left Arm Position", armLeft.getCurrentPosition());
                Telemetry.Item armRightPosition = telemetry.addData("Right Arm Position", armRight.getCurrentPosition());


                //code before waitForStart is run when Init button is pressed
                while(!isStarted){
                    //print encoder counts to telemetry while we manually move the arm
                    armLeftPosition.setValue(armLeft.getCurrentPosition());
                    telemetry.update();
                }

                //code after waitForStart is run when the start button is pressed


                int armTarget = 0;
                double armSpeed = 0;
                String armCurrentDirection = 'up';

                while (isOpModeActive()) {

                    /**
                     * BEGIN ARM LIFT

                     * Gamepad 1 btn A - arm lift up
                     * Gamepad 1 btn B - arm lift down
                     *
                     * !!! concerned about reliability of encoder count as it doesn't seem to be tracking accurately
                     **/

                    if (gamepad1.a){ // Arm UP
                        armTarget = 200;
                        armSpeed = 0.98;
                        armCurrentDirection = 'up';

                        armLeft.setPower(armSpeed);
                        armLeft.setTargetPosition(armTarget);

                    } else if (gamepad1.b){ // Arm DOWN
                        armTarget = 0;
                        armSpeed = -0.1;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
                        armCurrentDirection = 'down';

                        armLeft.setPower(armSpeed);
                        armLeft.setTargetPosition(armTarget);
                    }

                    // Remove Power from the Arm Motor if motor is close to 0 position, arm should drop
                    if ( armCurrentDirection === 'down' && ( armLeft.getTargetPosition() < 5 || armRight.getTargetPosition() < 5) ){
                        armSpeed = 0;
                        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                    /** END ARM LIFT **/


                    idle();

                    // Arm Lift Telemetry
                    if( armLeft.isBusy() ){
                        armLeftPosition.setValue(armLeft.getCurrentPosition());
                        telemetry.update();


            }
        }
    }
    public double PIDControl(double reference, double state, ElapsedTime timer ){
        double lastError=0.0;
        double error = reference - state;
        double integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return state;
    }
}
