package com.example.acraas.mavlink

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

// Unghiurile sunt in radiani, vitezele in NED, m/s
// Angles are in radians, velocities are NED, m/s
data class VehicleState(val rollRad: Float, val pitchRad: Float, val yawRad: Float,
                        val velocityNorthMps: Float, val velocityEastMps: Float, val velocityDownMps: Float)

// Camera priveste in sus pe axa de thrust. Dreapta imaginii = body +Y, jos = +X (FRD)
// Camera looks up the thrust axis. Image right = body +Y, down = +X (FRD)
class ThrustAxisController(hoverThrust: Float = HOVER_THRUST, private val thrustToWeight: Double = THRUST_TO_WEIGHT) {
    // Thrust-ul ramane fix aici, nu avem altitude hold
    // Thrust stays fixed here, there is no altitude hold
    val thrust = (hoverThrust * thrustToWeight).toFloat()

    fun update(angleXDeg: Float, angleYDeg: Float, state: VehicleState?): BodyRateCommand {
        val los = normalized(doubleArrayOf(tan(Math.toRadians(angleYDeg.toDouble())),
            tan(Math.toRadians(angleXDeg.toDouble())), -1.0))
        val axis = if (state == null) los else desiredAxis(los, state)

        val up = doubleArrayOf(0.0, 0.0, -1.0)
        val cross = doubleArrayOf(up[1] * axis[2] - up[2] * axis[1], up[2] * axis[0] - up[0] * axis[2], 0.0)
        val sinAngle = hypot(cross[0], cross[1])
        val angle = atan2(sinAngle, dot(up, axis))
        var rollRateRadS = 0.0
        var pitchRateRadS = 0.0
        if (sinAngle > 1e-9) {
            rollRateRadS = POINTING_GAIN * angle * cross[0] / sinAngle
            pitchRateRadS = POINTING_GAIN * angle * cross[1] / sinAngle
        }
        val rate = hypot(rollRateRadS, pitchRateRadS)
        if (rate > MAX_BODY_RATE_RAD_S) {
            rollRateRadS *= MAX_BODY_RATE_RAD_S / rate
            pitchRateRadS *= MAX_BODY_RATE_RAD_S / rate
        }

        // Adun 0.0 ca sa nu ramana -0.0 in comanda
        // Add 0.0 so the command does not keep -0.0
        return BodyRateCommand(rollRateRadS = (0.0 + rollRateRadS).toFloat(), pitchRateRadS = (0.0 + pitchRateRadS).toFloat(),
            yawRateRadS = 0f, thrust = thrust)
    }

    private fun desiredAxis(losBody: DoubleArray, state: VehicleState): DoubleArray {
        val r = bodyToNed(state.rollRad.toDouble(), state.pitchRad.toDouble(), state.yawRad.toDouble())
        val los = multiply(r, losBody)
        val velocity = doubleArrayOf(state.velocityNorthMps.toDouble(), state.velocityEastMps.toDouble(),
            state.velocityDownMps.toDouble())
        val gravity = doubleArrayOf(0.0, 0.0, GRAVITY)
        val thrustAccel = thrustToWeight * GRAVITY

        // Compensez gravitatia laterala si reduc deriva perpendiculara pe directia tintei
        // Cancel sideways gravity and damp drift across the target direction
        val cross = DoubleArray(3) { i ->
            -SIDE_VELOCITY_GAIN * (velocity[i] - dot(velocity, los) * los[i]) - (gravity[i] - dot(gravity, los) * los[i])
        }
        val limit = thrustAccel * sin(MAX_AXIS_OFFSET_RAD)
        val magnitude = sqrt(dot(cross, cross))
        if (magnitude > limit) for (i in 0..2) cross[i] *= limit / magnitude
        val along = sqrt(thrustAccel * thrustAccel - dot(cross, cross))
        val axisNed = normalized(DoubleArray(3) { along * los[it] + cross[it] })
        return multiplyTransposed(r, axisNed)
    }

    private fun bodyToNed(rollRad: Double, pitchRad: Double, yawRad: Double): Array<DoubleArray> {
        val (cr, sr) = cos(rollRad) to sin(rollRad)
        val (cp, sp) = cos(pitchRad) to sin(pitchRad)
        val (cy, sy) = cos(yawRad) to sin(yawRad)
        return arrayOf(
            doubleArrayOf(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            doubleArrayOf(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            doubleArrayOf(-sp, cp * sr, cp * cr))
    }

    private fun multiply(m: Array<DoubleArray>, v: DoubleArray) = DoubleArray(3) { dot(m[it], v) }
    private fun multiplyTransposed(m: Array<DoubleArray>, v: DoubleArray) =
        DoubleArray(3) { m[0][it] * v[0] + m[1][it] * v[1] + m[2][it] * v[2] }
    private fun dot(a: DoubleArray, b: DoubleArray) = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    private fun normalized(v: DoubleArray): DoubleArray {
        val n = sqrt(dot(v, v))
        return DoubleArray(3) { v[it] / n }
    }

    companion object {
        // Valoarea MPC_THR_HOVER pentru Tadpole-ul calibrat
        // MPC_THR_HOVER for the calibrated Tadpole
        const val HOVER_THRUST = 0.17872092f
        const val THRUST_TO_WEIGHT = 2.0

        // Gain-urile de pointing si damping sunt in 1/s
        // Pointing and damping gains are in 1/s
        const val POINTING_GAIN = 5.0

        const val SIDE_VELOCITY_GAIN = 1.5

        val MAX_AXIS_OFFSET_RAD = Math.toRadians(30.0)
        const val MAX_BODY_RATE_RAD_S = 4.0
        const val GRAVITY = 9.80665

        const val STATE_TIMEOUT_MS = 300L
    }
}
