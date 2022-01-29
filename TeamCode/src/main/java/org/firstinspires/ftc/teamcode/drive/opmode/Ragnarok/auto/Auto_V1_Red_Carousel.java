package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.HardwareRagnarok;
import org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.PoseStorage;

@Config
@Autonomous(name="--Main-- Auto V1 (Red Carousel)", group="drive")
public class Auto_V1_Red_Carousel extends LinearOpMode {
    public static double drigh1 = 8;
    public static double tforw1 = 3;
    public static double dforw1 = 8;
    public static double drigh2 = 26;
    public static double dforw2 = 10;


    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareRagnarok robot   = new HardwareRagnarok();
        robot.init(hardwareMap);

        Pose2d startPose = new Pose2d(-42, -63, Math.PI/2);

        drive.setPoseEstimate(startPose);

        Trajectory righ1 = drive.trajectoryBuilder(startPose)
                .strafeRight(drigh1)
                .build();

        Trajectory forw1 = drive.trajectoryBuilder(righ1.end())
                .forward(dforw1)
                .build();

        Trajectory righ2 = drive.trajectoryBuilder(forw1.end())
                .strafeRight(drigh2)
                .build();

        Trajectory forw2 = drive.trajectoryBuilder(righ2.end())
                .forward(dforw2)
                .build();


        // forward, spin, right, forward

        waitForStart();


        sleep(500);

        drive.followTrajectory(righ1);

        drive.setMotorPowers(0.2,0.2,0.2,0.2);
        sleep(Math.round(tforw1*1000));
        drive.setMotorPowers(0,0,0,0);

        robot.spinny_thing.setPower(0.25);
        sleep(6000);
        robot.spinny_thing.setPower(0);

        drive.followTrajectory(righ2);
        drive.followTrajectory(forw2);


        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
