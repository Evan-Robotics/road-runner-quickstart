package org.firstinspires.ftc.teamcode.drive.opmode.Ragnarok.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name="TEST")
class KotlinTest : LinearOpMode() {
    val left: DcMotor? = hardwareMap.get(DcMotor::class.java, "LEFT")
    val right: DcMotor? = hardwareMap.get(DcMotor::class.java, "RIGHT")

    override fun runOpMode() {

    }

}