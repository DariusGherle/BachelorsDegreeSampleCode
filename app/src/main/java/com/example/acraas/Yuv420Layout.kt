package com.example.acraas

import java.nio.ByteBuffer

// Chroma poate fi planar sau interleaved. Tin cont de stride si pozitia buffer-ului
// Chroma can be planar or interleaved. Account for stride and buffer position
object Yuv420Layout {
    data class Plane(val buffer: ByteBuffer, val rowStride: Int, val pixelStride: Int)

    fun toNv21(width: Int, height: Int, planes: List<Plane>): ByteArray {
        require(width > 0 && height > 0 && width % 2 == 0 && height % 2 == 0 && planes.size == 3)
        val result = ByteArray(width * height * 3 / 2)
        fun copy(plane: Plane, rows: Int, columns: Int, offset: Int, stride: Int) {
            val base = plane.buffer.position()
            for (row in 0 until rows) for (column in 0 until columns) {
                result[offset + (row * columns + column) * stride] =
                    plane.buffer.get(base + row * plane.rowStride + column * plane.pixelStride)
            }
        }
        copy(planes[0], height, width, 0, 1)
        copy(planes[2], height / 2, width / 2, width * height, 2)
        copy(planes[1], height / 2, width / 2, width * height + 1, 2)
        return result
    }
}
