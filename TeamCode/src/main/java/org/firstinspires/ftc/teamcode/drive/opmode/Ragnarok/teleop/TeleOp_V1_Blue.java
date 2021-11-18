package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;

@TeleOp(name = "--Alpha-- TeleOp V1", group = "Ragnarok")
public class TeleOp_V1_Blue extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 1;

    // Driving variables
    enum Mode {
        NORMAL_CONTROL,
        TANK_CONTROL,
        GRID          // testing
    }

    boolean spinny_direction = false;
    boolean spinny_direction_toggler = false;

    double speedChange1;

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    //Target Position of Storage Unit
    private Vector2d storage_pos = new Vector2d(-66, 37);

    //Timers
    private final ElapsedTime emergencytimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializations
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot = new HardwareRagnarok();
        robot.init(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Start Pose from PoseStorage      Left tape: (new Pose2d(-63,50));
        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);


        waitForStart();

        ////VARIABLES\\\\\
        // (for servos, when we have them)

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            //Gamepad Input
            double ly;
            double lx;
            double rx;

            //Initialize Localizer
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            Pose2d driveDirection = new Pose2d();

            //Initialize FTC Dashboard
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            //TelemetryPacket packet = new TelemetryPacket();

            //// Gamepad Controls \\\\

            // Speed change
            if (gamepad1.left_bumper) {
                speedChange1 = 0.5;
            } else if (gamepad1.right_bumper) {
                speedChange1 = 1;
            } else {
                speedChange1 = 0.7;
            }

            // y toggles direction of spinny thing
            if (spinny_direction_toggler && !gamepad1.y) {
                spinny_direction = !spinny_direction;
            }
            spinny_direction_toggler = gamepad1.y;

            robot.spinny_thing.setPower(spinny_direction ? speedChange1 * gamepad1.left_trigger : -1 * speedChange1 * gamepad1.left_trigger);

            // right trigger controls intake
            robot.intake_main.setPower(gamepad1.right_trigger * speedChange1);
            robot.intake2.setPower(gamepad1.right_trigger * speedChange1 * 0.7);


            if (gamepad1.start) {
                drive.getLocalizer().setPoseEstimate(new Pose2d(-63, 60, Math.PI/2));
            }

            //Distance to Tower
            double getDistance = Math.sqrt(Math.pow(storage_pos.getX() - poseEstimate.getX(), 2) + Math.pow(storage_pos.getY() - poseEstimate.getY(), 2));

            switch (currentMode) {
                case NORMAL_CONTROL:

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    //Normal Robot Control
                    driveDirection = new Pose2d(
                            Math.signum(ly) * ly * ly * speedChange1,
                            Math.signum(lx) * lx * lx * speedChange1,
                            Math.signum(rx) * rx * rx * speedChange1
                    );

                    // Switch to tank if gamepad1 left dpad is activated
                    if (gamepad1.dpad_left) {
                        currentMode = Mode.GRID;
                    }
                    break;

                case GRID: // Switch to normal control if gamepad1 dpad right is activated

                    if (gamepad1.dpad_right) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    ly = -gamepad1.left_stick_y;
                    lx = -gamepad1.left_stick_x;
                    rx = -gamepad1.right_stick_x;

                    Vector2d fieldFrameInput = new Vector2d(
                            Math.signum(ly) * Math.pow(ly, 2) * speedChange1,
                            Math.signum(lx) * Math.pow(lx, 2) * speedChange1
                    );

                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
                    //headingController.setTargetPosition(rx * speedChange1);

                    double headingInput = Math.signum(rx) * Math.pow(rx, 2) * speedChange1;

                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    if (inObstacle(poseEstimate.plus(driveDirection))) {
                        telemetry.addData("Collision", "Detected");
                    } else {
                        telemetry.addData("Collision", "Not Detected");
                    }

                    break;

            }

            //Dashboard View
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
            drive.setWeightedDrivePower(driveDirection);
            headingController.update(poseEstimate.getHeading());
            drive.getLocalizer().update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            //Driver Station Telemetry

            //telemetry.addData("Alpha", robot.color_sensor.alpha());
            telemetry.addData("mode", currentMode);
            telemetry.addData("Spinner Direction", spinny_direction ? "Red" : "Blue");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private boolean inObstacle(Pose2d pos) {
        if ((pos.getX() > 18 || pos.getX() < 30) && (pos.getY() > -54 || pos.getY() < 54)) {
            return true;
        }
        if ((pos.getX() > 24 || pos.getX() < 54) && (pos.getY() > -30 || pos.getY() < -18)) {
            return true;
        }

        return false;
    }
}
