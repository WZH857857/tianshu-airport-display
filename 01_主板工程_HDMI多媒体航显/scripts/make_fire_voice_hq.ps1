# Synthesise a clean 16 kHz fire-evacuation announcement with Windows TTS.
#
# The FPGA stream parser and voice player already accept 16 kHz, so keeping
# the TTS output at its native rate avoids the aliasing produced by asking
# SAPI to render directly at 8 kHz.
#
#   powershell -File scripts\make_fire_voice_hq.ps1 `
#       -Voice "Microsoft Kangkang" -Rate 0 -Out build\fire_voice_hq_male.wav
#   python scripts\make_fire_voice_hq_asset.py `
#       build\fire_voice_hq_male.wav sdcard\FIRE.WAV

param(
    [int]    $Rate = 0,
    [int]    $Volume = 100,
    [string] $Voice = 'Microsoft Kangkang',
    [string] $Out = 'build\fire_voice_hq_male.wav'
)

$Text = @'
紧急通知。紧急通知。本区域发生火情。
请各位旅客保持冷静，不要惊慌，听从现场工作人员的指挥。
请立即从最近的安全出口有序撤离。
请勿乘坐电梯，不要携带行李，不要拥挤。
撤离时请照顾好老人和儿童，迅速前往安全区域。
'@

Add-Type -AssemblyName System.Speech

$path = if ([System.IO.Path]::IsPathRooted($Out)) {
    $Out
} else {
    Join-Path (Get-Location).Path $Out
}

$parent = Split-Path -Parent $path
if ($parent -and !(Test-Path -LiteralPath $parent)) {
    New-Item -ItemType Directory -Path $parent -Force | Out-Null
}

$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$synth.SelectVoice($Voice)
$synth.Rate = $Rate
$synth.Volume = $Volume

$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
    16000,
    [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
    [System.Speech.AudioFormat.AudioChannel]::Mono)

$synth.SetOutputToWaveFile($path, $fmt)
$synth.Speak($Text)
$synth.SetOutputToNull()
$synth.Dispose()

$bytes = (Get-Item -LiteralPath $path).Length
Write-Host ("{0}: {1} bytes, {2:N2} s at 16 kHz" -f `
    $path, $bytes, (($bytes - 44) / 32000.0))
