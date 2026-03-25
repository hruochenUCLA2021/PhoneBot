package com.example.phonebot_app_android.llm

import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.DataOutputStream
import java.io.File
import java.net.HttpURLConnection
import java.net.URL
import java.nio.charset.StandardCharsets
import java.util.UUID

data class LlmResponse(
    val transcript: String,
    val reply: String,
)

data class PiperVoice(
    val voiceId: String,
    val label: String,
)

object LlmHttpClient {
    fun postText(baseUrl: String, text: String): LlmResponse {
        val url = URL("${baseUrl.trimEnd('/')}/api/llm/text")
        val conn = (url.openConnection() as HttpURLConnection)
        conn.requestMethod = "POST"
        conn.connectTimeout = 10_000
        conn.readTimeout = 120_000
        conn.doOutput = true
        conn.setRequestProperty("Content-Type", "application/json; charset=utf-8")

        val body = JSONObject().put("text", text).toString()
        conn.outputStream.use { os ->
            os.write(body.toByteArray(StandardCharsets.UTF_8))
        }

        val code = conn.responseCode
        val bytes =
            (if (code in 200..299) conn.inputStream else conn.errorStream).use { it.readBytes() }
        val s = String(bytes, StandardCharsets.UTF_8)
        if (code !in 200..299) error("HTTP $code: $s")
        val j = JSONObject(s)
        return LlmResponse(
            transcript = j.optString("transcript", ""),
            reply = j.optString("reply", ""),
        )
    }

    fun postAudio(baseUrl: String, wavFile: File, whisperModel: String = "base", whisperLanguage: String = "en"): LlmResponse {
        val url = URL("${baseUrl.trimEnd('/')}/api/llm/audio")
        val boundary = "----PhoneBotBoundary${UUID.randomUUID()}"
        val conn = (url.openConnection() as HttpURLConnection)
        conn.requestMethod = "POST"
        conn.connectTimeout = 10_000
        conn.readTimeout = 300_000
        conn.doOutput = true
        conn.setRequestProperty("Content-Type", "multipart/form-data; boundary=$boundary")

        DataOutputStream(conn.outputStream).use { out ->
            fun writeFormField(name: String, value: String) {
                out.writeBytes("--$boundary\r\n")
                out.writeBytes("Content-Disposition: form-data; name=\"$name\"\r\n\r\n")
                out.writeBytes(value)
                out.writeBytes("\r\n")
            }

            fun writeFileField(name: String, file: File, contentType: String) {
                out.writeBytes("--$boundary\r\n")
                out.writeBytes("Content-Disposition: form-data; name=\"$name\"; filename=\"${file.name}\"\r\n")
                out.writeBytes("Content-Type: $contentType\r\n\r\n")
                file.inputStream().use { it.copyTo(out) }
                out.writeBytes("\r\n")
            }

            writeFormField("whisper_model", whisperModel)
            writeFormField("whisper_language", whisperLanguage)
            writeFileField("audio", wavFile, "audio/wav")

            out.writeBytes("--$boundary--\r\n")
            out.flush()
        }

        val code = conn.responseCode
        val bytes =
            (if (code in 200..299) conn.inputStream else conn.errorStream).use { it.readBytes() }
        val s = String(bytes, StandardCharsets.UTF_8)
        if (code !in 200..299) error("HTTP $code: $s")
        val j = JSONObject(s)
        return LlmResponse(
            transcript = j.optString("transcript", ""),
            reply = j.optString("reply", ""),
        )
    }

    fun getPiperVoices(baseUrl: String, langTag: String): List<PiperVoice> {
        val url =
            URL(
                "${baseUrl.trimEnd('/')}/api/tts/voices?lang_tag=${
                    java.net.URLEncoder.encode(langTag, "UTF-8")
                }",
            )
        val conn = (url.openConnection() as HttpURLConnection)
        conn.requestMethod = "GET"
        conn.connectTimeout = 10_000
        conn.readTimeout = 30_000
        val code = conn.responseCode
        val bytes =
            (if (code in 200..299) conn.inputStream else conn.errorStream).use { it.readBytes() }
        val s = String(bytes, StandardCharsets.UTF_8)
        if (code !in 200..299) error("HTTP $code: $s")
        val j = JSONObject(s)
        val arr = j.optJSONArray("voices")
        val out = ArrayList<PiperVoice>()
        if (arr != null) {
            for (i in 0 until arr.length()) {
                val o = arr.optJSONObject(i) ?: continue
                val id = o.optString("voice_id", "")
                val label = o.optString("label", id)
                if (id.isNotBlank()) out.add(PiperVoice(voiceId = id, label = label))
            }
        }
        return out
    }

    fun postPiperTts(baseUrl: String, text: String, langTag: String, voiceId: String?): ByteArray {
        val url = URL("${baseUrl.trimEnd('/')}/api/tts/speak")
        val conn = (url.openConnection() as HttpURLConnection)
        conn.requestMethod = "POST"
        conn.connectTimeout = 10_000
        conn.readTimeout = 300_000
        conn.doOutput = true
        conn.setRequestProperty("Content-Type", "application/json; charset=utf-8")

        val body =
            JSONObject()
                .put("text", text)
                .put("lang_tag", langTag)
                .also { j ->
                    if (!voiceId.isNullOrBlank()) j.put("voice_id", voiceId)
                }
                .toString()
        conn.outputStream.use { os ->
            os.write(body.toByteArray(StandardCharsets.UTF_8))
        }

        val code = conn.responseCode
        val bytes =
            (if (code in 200..299) conn.inputStream else conn.errorStream).use { it.readBytes() }
        if (code !in 200..299) {
            val s = String(bytes, StandardCharsets.UTF_8)
            error("HTTP $code: $s")
        }
        return bytes
    }
}

