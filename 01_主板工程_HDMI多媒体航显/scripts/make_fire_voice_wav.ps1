# Synthesise the fire alarm announcement with Windows TTS.
#
#   powershell -File scripts\make_fire_voice_wav.ps1 -Out build\fire_voice_raw.wav
#   python scripts\make_fire_voice_asset.py build\fire_voice_raw.wav sdcard\FIRE.WAV 10
#
# The text is embedded here so the Chinese prompt survives the shell unchanged.
# FIRE.WAV is looped by the FPGA while the latched fire alarm is active.
param(
    [int]    $Rate = 1,          # SAPI rate, -10..10
    [int]    $Volume = 100,
    [string] $Voice = 'Microsoft Huihui Desktop',
    [string] $Out = 'fire_voice_raw.wav'
)

$Text = @'
紧急通知，紧急通知，本区域发生火情。请各位旅客保持冷静，听从工作人员指挥，立即从最近的安全出口有序撤离。
请勿乘坐电梯，不要携带行李，不要拥挤，迅速撤离。
'@

Add-Type -AssemblyName System.Speech

$path = if ([System.IO.Path]::IsPathRooted($Out)) { $Out }
        else { Join-Path (Get-Location).Path $Out }

$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$synth.SelectVoice($Voice)
$synth.Rate = $Rate
$synth.Volume = $Volume

$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(
    8000,
    [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,
    [System.Speech.AudioFormat.AudioChannel]::Mono)

$synth.SetOutputToWaveFile($path, $fmt)
$synth.Speak($Text)
$synth.SetOutputToNull()
$synth.Dispose()

$bytes = (Get-Item -LiteralPath $path).Length
Write-Host ("{0}: {1} bytes, {2:N2} s of speech" -f $path, $bytes, (($bytes - 44) / 16000.0))
