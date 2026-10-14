# Synthesise the spoken part of the SD card announcement with Windows TTS.
#
#   powershell -File scripts\make_voice_wav.ps1 -Out raw.wav
#   python scripts\make_voice_asset.py raw.wav sdcard\VOICE.WAV stream 60
#
# The second step trims the silence, prepends the boarding chime, and checks
# the requested duration.  The streaming player has no 40960 sample bank
# limit, so a one minute or longer airport announcement is allowed.
#
# Voices installed on this machine: Microsoft Huihui Desktop (zh-CN, female),
# Microsoft Kangkang (zh-CN, male), Microsoft Yaoyao (zh-CN, female).
param(
    [string] $Text = '',
    [int]    $Rate = 1,          # SAPI rate, -10..10
    [int]    $Volume = 100,
    [string] $Voice = 'Microsoft Huihui Desktop',
    [string] $Out = 'voice_raw.wav'
)

if ([string]::IsNullOrWhiteSpace($Text)) {
    $Text = @'
尊敬的旅客，欢迎来到天枢空港。下面为您播报航班动态、安全检查和服务提示，请留意相关信息。

前往北京首都国际机场的旅客请注意，您乘坐的CA1234次航班现在开始登机。请携带好登机牌和随身行李，从三号登机口依次排队登机。需要优先登机的旅客，请提前告知工作人员，我们将为您提供协助。

前往上海浦东国际机场的旅客请注意，您乘坐的MU5678次航班登机口已经变更，由原来的五号登机口调整到九号登机口。请携带随身物品，按照机场指示标识前往新的登机口。给您带来不便，敬请谅解。

前往广州白云国际机场的旅客请注意，您乘坐的CZ2468次航班受天气影响，起飞时间预计延误三十分钟。请您在候机区休息，留意广播和航班显示屏上的最新信息。不要远离登机口，登机时间如有变化，我们将及时通知。

前往深圳宝安国际机场的旅客请注意，您乘坐的ZH1357次航班即将开始登机，请在四号登机口排队等候。请出示登机牌，按照座位区域顺序登机。携带儿童和老人的旅客，可以优先办理登机。

最后提醒各位旅客，请看管好您的随身物品，不要将行李交给陌生人保管。请留意航班显示屏和广播通知，提前到达登机口。祝您旅途愉快，一路平安。谢谢您的配合。
'@
}

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
