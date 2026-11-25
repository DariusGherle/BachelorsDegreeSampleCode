package com.example.acraas

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.graphics.Matrix
import android.os.Bundle
import android.os.SystemClock
import android.util.Log
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.camera.core.*
import androidx.camera.lifecycle.ProcessCameraProvider
import androidx.core.content.ContextCompat
import com.example.acraas.databinding.ActivityMainBinding
import com.example.acraas.mavlink.DroneController
import com.example.acraas.mavlink.ThrustAxisController
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors

class MainActivity : AppCompatActivity(), Detector.DetectorListener, DroneController.DroneListener {
    private lateinit var binding: ActivityMainBinding
    private val isFrontCamera = false

    private var preview: Preview? = null
    private var imageAnalyzer: ImageAnalysis? = null
    private var camera: Camera? = null
    private var cameraProvider: ProcessCameraProvider? = null
    private var detector: Detector? = null
    private var droneController: DroneController? = null

    private lateinit var cameraExecutor: ExecutorService
    private lateinit var yuvToRgbConverter: YuvToRgbConverter

    private var trackingEnabledAtMs = 0L
    @Volatile private var cameraActive = false
    private var isTrackingActive = false
    private var targetObject: BoundingBox? = null
    private val thrustAxisController = ThrustAxisController()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        cameraExecutor = Executors.newSingleThreadExecutor()
        yuvToRgbConverter = YuvToRgbConverter(this)

        cameraExecutor.execute {
            detector = Detector(baseContext, Constants.MODEL_PATH, Constants.LABELS_PATH, this) {
                toast(it)
            }
        }

        initializeDroneController()

        bindListeners()
    }

    private fun initializeDroneController() {
        droneController = DroneController(this, this)
        droneController?.initialize()
    }

    private fun bindListeners() {
        binding.isGpu.setOnCheckedChangeListener { buttonView, isChecked ->
            cameraExecutor.submit {
                detector?.restart(isGpu = isChecked)
            }
            if (isChecked) {
                buttonView.setBackgroundColor(ContextCompat.getColor(this, R.color.orange))
            } else {
                buttonView.setBackgroundColor(ContextCompat.getColor(this, android.R.color.darker_gray))
            }
        }

        try {
            binding.trackingSwitch.setOnCheckedChangeListener { _, isChecked ->
                isTrackingActive = isChecked
                trackingEnabledAtMs = SystemClock.elapsedRealtime()
                droneController?.setTrackingEnabled(isChecked)
                try {
                    binding.overlay.setTrackingActive(isChecked)
                } catch (e: Exception) {
                    Log.w(TAG, "OverlayView nu are setTrackingActive: ${e.message}")
                }

                if (isChecked) {
                    toast("Tracking activat - Se asteapta detectarea tintei")
                } else {
                    targetObject = null
                    toast("Tracking dezactivat")
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "trackingSwitch nu exista in layout: ${e.message}")
        }

        try {
            binding.connectDroneButton.setOnClickListener {
                if (droneController?.isConnected() == true) {
                    droneController?.disconnect()
                } else {
                    droneController?.initialize()
                }
            }
        } catch (e: Exception) {
            Log.w(TAG, "connectDroneButton nu exista in layout: ${e.message}")
        }
    }

    private fun startCamera() {
        if (!cameraActive) return
        val cameraProviderFuture = ProcessCameraProvider.getInstance(this)
        cameraProviderFuture.addListener({
            if (!cameraActive) return@addListener
            cameraProvider = cameraProviderFuture.get()
            bindCameraUseCases()
        }, ContextCompat.getMainExecutor(this))
    }

    @androidx.annotation.OptIn(ExperimentalGetImage::class)
    private fun bindCameraUseCases() {
        val cameraProvider = cameraProvider ?: throw IllegalStateException("Camera initialization failed")
        val rotation = binding.viewFinder.display.rotation

        val cameraSelector = CameraSelector.Builder()
            .requireLensFacing(CameraSelector.LENS_FACING_BACK)
            .build()

        preview = Preview.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setTargetRotation(rotation)
            .build()

        imageAnalyzer = ImageAnalysis.Builder()
            .setTargetAspectRatio(AspectRatio.RATIO_4_3)
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .setTargetRotation(rotation)
            .setOutputImageFormat(ImageAnalysis.OUTPUT_IMAGE_FORMAT_YUV_420_888)
            .build()

        imageAnalyzer?.setAnalyzer(cameraExecutor) { imageProxy ->
            val observedAtMs = SystemClock.elapsedRealtime()
            var bitmap: Bitmap? = null
            var rotatedBitmap: Bitmap? = null
            try {
                if (!cameraActive) return@setAnalyzer
                val image = imageProxy.image ?: return@setAnalyzer
                val frame = Bitmap.createBitmap(imageProxy.width, imageProxy.height, Bitmap.Config.ARGB_8888)
                bitmap = frame
                yuvToRgbConverter.yuvToRgb(image, frame)
                val matrix = Matrix().apply {
                    postRotate(imageProxy.imageInfo.rotationDegrees.toFloat())
                    if (isFrontCamera) postScale(-1f, 1f)
                }
                val rotated = Bitmap.createBitmap(frame, 0, 0, frame.width, frame.height, matrix, true)
                rotatedBitmap = rotated
                detector?.detect(rotated, observedAtMs)
            } catch (e: Exception) {
                Log.e(TAG, "Camera/detector processing failed", e)
            } finally {
                if (rotatedBitmap !== bitmap) rotatedBitmap?.recycle()
                bitmap?.recycle()
                imageProxy.close()
            }
        }

        cameraProvider.unbindAll()

        try {
            camera = cameraProvider.bindToLifecycle(this, cameraSelector, preview, imageAnalyzer)
            preview?.setSurfaceProvider(binding.viewFinder.surfaceProvider)
        } catch (exc: Exception) {
            Log.e(TAG, "Use case binding failed", exc)
        }
    }

    private fun allPermissionsGranted() = REQUIRED_PERMISSIONS.all {
        ContextCompat.checkSelfPermission(baseContext, it) == PackageManager.PERMISSION_GRANTED
    }

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) {
        if (it[Manifest.permission.CAMERA] == true) startCamera()
    }

    private fun toast(message: String) {
        runOnUiThread {
            Toast.makeText(baseContext, message, Toast.LENGTH_LONG).show()
        }
    }

    override fun onPause() {
        cameraActive = false
        binding.trackingSwitch.isChecked = false
        droneController?.setTrackingEnabled(false)
        imageAnalyzer?.clearAnalyzer()
        cameraProvider?.unbindAll()
        super.onPause()
    }

    override fun onDestroy() {
        droneController?.cleanup()

        // Inchid GPU-ul pe acelasi executor, dupa frame-urile deja in lucru
        // Close the GPU on the same executor, after frames already in progress
        cameraExecutor.execute {
            try { detector?.close() } finally { yuvToRgbConverter.close() }
        }
        cameraExecutor.shutdown()
        super.onDestroy()
    }

    override fun onResume() {
        super.onResume()
        cameraActive = true
        if (allPermissionsGranted()) {
            startCamera()
        } else {
            requestPermissionLauncher.launch(REQUIRED_PERMISSIONS)
        }
    }

    override fun onEmptyDetect() {
        runOnUiThread {
            if (cameraActive) binding.overlay.clear()
        }
    }

    override fun onDetect(boundingBoxes: List<BoundingBox>, inferenceTime: Long, observedAtMs: Long) {
        runOnUiThread {
            if (!cameraActive) return@runOnUiThread
            binding.inferenceTime.text = "${inferenceTime}ms"

            val screenCenterX = binding.viewFinder.width / 2f
            val screenCenterY = binding.viewFinder.height / 2f

            var closestObject: BoundingBox? = null
            var minDistance = Float.MAX_VALUE

            boundingBoxes.forEach { box ->
                val boxCenterX = (box.x1 + box.x2) / 2f * binding.viewFinder.width
                val boxCenterY = (box.y1 + box.y2) / 2f * binding.viewFinder.height

                val deltaX = boxCenterX - screenCenterX
                val deltaY = boxCenterY - screenCenterY
                val distance = kotlin.math.sqrt((deltaX * deltaX + deltaY * deltaY).toDouble()).toFloat()

                if (distance < minDistance) {
                    minDistance = distance
                    closestObject = box
                }
            }

            if (isTrackingActive && observedAtMs >= trackingEnabledAtMs && closestObject != null) {
                targetObject = closestObject

                sendTargetCommandToDrone(closestObject!!, observedAtMs)
            }

            binding.overlay.setResults(boundingBoxes)
        }
    }

    private fun sendTargetCommandToDrone(target: BoundingBox, observedAtMs: Long) {
        val screenCenterX = binding.viewFinder.width / 2f
        val screenCenterY = binding.viewFinder.height / 2f
        val focalLengthPx = 1000f

        val boxCenterX = (target.x1 + target.x2) / 2f * binding.viewFinder.width
        val boxCenterY = (target.y1 + target.y2) / 2f * binding.viewFinder.height

        val deltaX = boxCenterX - screenCenterX
        val deltaY = boxCenterY - screenCenterY

        val angleXDeg = Math.toDegrees(kotlin.math.atan2(deltaX.toDouble(), focalLengthPx.toDouble())).toFloat()
        val angleYDeg = Math.toDegrees(kotlin.math.atan2(deltaY.toDouble(), focalLengthPx.toDouble())).toFloat()

        val command = thrustAxisController.update(angleXDeg, angleYDeg, droneController?.vehicleState())
        val bodyRollRateRadS = command.rollRateRadS
        val bodyPitchRateRadS = command.pitchRateRadS
        val bodyYawRateRadS = command.yawRateRadS
        val thrust = command.thrust

        droneController?.sendBodyRateCommand(
            rollRateRadS = bodyRollRateRadS,
            pitchRateRadS = bodyPitchRateRadS,
            yawRateRadS = bodyYawRateRadS,
            thrust = thrust,
            observedAtMs = observedAtMs
        )

        updateUI("Target: ${target.clsName} | Angles: %.1f°, %.1f° | Rates: R=%.3f, P=%.3f"
            .format(angleXDeg, angleYDeg, bodyRollRateRadS, bodyPitchRateRadS))

        Log.d("DroneTracking", "Attitude control - ${target.clsName}: angleXDeg=$angleXDeg°, angleYDeg=$angleYDeg° -> rollRateRadS=$bodyRollRateRadS, pitchRateRadS=$bodyPitchRateRadS")
    }

    private fun updateUI(message: String) {
        runOnUiThread {
            try {
                binding.targetInfo.text = message
            } catch (e: Exception) {
                Log.d(TAG, message)
            }
        }
    }

    override fun onDroneConnected() {
        runOnUiThread {
            try {
                binding.droneStatus.text = "Drona conectata"
                binding.connectDroneButton.text = "Deconecteaza"
                binding.trackingSwitch.isEnabled = true
            } catch (e: Exception) {
                Log.i(TAG, "Drona conectata")
                toast("Drona conectata")
            }
        }
    }

    override fun onDroneDisconnected() {
        runOnUiThread {
            try {
                binding.droneStatus.text = "Drona deconectata"
                binding.connectDroneButton.text = "Conecteaza"
                binding.trackingSwitch.isEnabled = false
                binding.trackingSwitch.isChecked = false
                binding.offboardStatus.text = "Mod necunoscut - deconectat"
            } catch (e: Exception) {
                Log.i(TAG, "Drona deconectata")
                toast("Drona deconectata")
            }
            isTrackingActive = false
        }
    }

    override fun onDroneError(error: String) {
        runOnUiThread {
            try {
                binding.droneStatus.text = "Eroare: $error"
            } catch (e: Exception) {
                Log.e(TAG, "Eroare drona: $error")
            }
            toast("Eroare drona: $error")
        }
    }

    override fun onOffboardModeChanged(isActive: Boolean) {
        runOnUiThread {
            try {
                binding.offboardStatus.text = if (isActive) "OFFBOARD ACTIV" else "OFFBOARD INACTIV"
            } catch (e: Exception) {
                Log.i(TAG, if (isActive) "OFFBOARD ACTIV" else "OFFBOARD INACTIV")
            }
        }
    }

    override fun onTrackingStopped(reason: String) {
        runOnUiThread {
            binding.trackingSwitch.isChecked = false
            isTrackingActive = false
            targetObject = null
            updateUI(reason)
        }
    }

    override fun onTelemetryReceived(attitude: DroneController.AttitudeData) {
        runOnUiThread {
            Log.d(TAG, "Telemetry - Roll: %.1f°, Pitch: %.1f°, Yaw: %.1f°, RollRate: %.1f, PitchRate: %.1f, YawRate: %.1f".format(
                Math.toDegrees(attitude.rollRad.toDouble()),
                Math.toDegrees(attitude.pitchRad.toDouble()),
                Math.toDegrees(attitude.yawRad.toDouble()),
                attitude.rollRateRadS,
                attitude.pitchRateRadS,
                attitude.yawRateRadS
            ))
        }
    }

    companion object {
        private const val TAG = "MainActivity"
        private val REQUIRED_PERMISSIONS = arrayOf(
            Manifest.permission.CAMERA
        )
    }
}
