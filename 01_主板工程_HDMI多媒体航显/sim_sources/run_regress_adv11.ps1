param(
    [string]   $Proj = 'E:\FPGA guo sai\jing jie yao qiu\hdmi_mm_player_advanced',
    [string[]] $Only = @(),
    [int]      $TimeoutSec = 1800
)

# iverilog lives in C:\iverilog on the older images and E:\iverilog on the
# current one; pick whichever is actually installed.
$ivRoot = if (Test-Path 'C:\iverilog\bin\iverilog.exe') { 'C:\iverilog\bin' }
          else { 'E:\iverilog\bin' }
$iv = Join-Path $ivRoot 'iverilog.exe'
$vv = Join-Path $ivRoot 'vvp.exe'
$out = Join-Path $Proj 'sim\regress_adv11'
New-Item -ItemType Directory -Force -Path $out | Out-Null

# Every self written RTL file, encrypted vendor blocks replaced by the stubs.
# hdmi_mm_player.v pulls seven files in through `include, so those must not be
# listed a second time or iverilog reports a duplicate module declaration.
$fromInclude = @('wav_voice_stream.v', 'voice_player_stream.v',
                 'voice_repeat_ctrl.v', 'bmp_scaler.v',
                 'uart_rx.v', 'uart_tx.v', 'uart_ctrl.v',
                 'i2c_rtc.v', 'emergency_alert.v')
$rtl = Get-ChildItem (Join-Path $Proj 'rtl') -Recurse -Filter *.v |
       Where-Object { ($_.Name -notmatch '\.enc\.') -and
                      ($fromInclude -notcontains $_.Name) } |
       ForEach-Object { $_.FullName }

$common = @(
    (Join-Path $Proj 'sim\check_stubs.v'),
    (Join-Path $Proj 'sim\sdr_as_ram_stub.v'),
    (Join-Path $Proj 'sim\sdram_model.v'),
    (Join-Path $Proj 'al_ip\video_pll.v'),
    (Join-Path $Proj 'al_ip\sys_pll.v')
)

$vendorDir = Join-Path $Proj 'vendor\lab_ex5_i2s\src\user_source\hdl_source'
foreach ($f in @('I2S_receiver.v', 'audio_arc_calculate.v', 'hdmi_phy_warpper.v', 'lane_lvds_10_1.v')) {
    $common += (Join-Path $vendorDir $f)
}

# ADV11: the serial command handler is checked first, then the whole ADV10
# set is re-run to prove the new include does not disturb the picture, the
# parameters or the audio path.
$tbs = @(
    'tb_uart_ctrl',
    'tb_flight_board',
    'tb_bmp_scale_chain',
    'tb_player_emg_release',
    'tb_frame_store_emg_slot',
    'tb_player_manual_auto',
    'tb_player_cache',
    'tb_player_key',
    'tb_player_key_immediate',
    'tb_player_key_restart_timer',
    'tb_player_swap_timeout',
    'tb_player_preload',
    'tb_param_control',
    'tb_param_overlay',
    'tb_resolution_osd',
    'tb_emergency_alert_seg119',
    'tb_fat32_file_reader',
    'tb_ui_overlay_adv01',
    'tb_audio_vis',
    'tb_voice_repeat_ctrl',
    'tb_voice_stream_player',
    'tb_voice_stream_parser',
    'tb_fire_wav_hq',
    'tb_ulaw',
    'tb_fade_ctrl',
    'tb_fade_timing'
)
if ($Only.Count -gt 0) { $tbs = $Only }

# tb_uart_ctrl instantiates uart_ctrl directly, so it needs the three serial
# files on its own command line; the other benches get them through the
# `include inside hdmi_mm_player.v.
$special = @{
    'tb_param_overlay' = @{
        Mode  = '-g2012'
        Files = @('sim\tb_param_overlay.v', 'rtl\ui_overlay.v',
                  'rtl\ui\flight_board.v')
    }
    'tb_resolution_osd' = @{
        Mode  = '-g2012'
        Files = @('sim\tb_resolution_osd.v', 'rtl\ui_overlay.v',
                  'rtl\ui\flight_board.v')
    }
    'tb_uart_ctrl' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_uart_ctrl.v', 'rtl\uart\uart_rx.v',
                  'rtl\uart\uart_tx.v', 'rtl\uart\uart_ctrl.v')
    }
    'tb_flight_board' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_flight_board.v', 'rtl\ui\flight_board.v')
    }
    # The player sequencer bench drives player_ctrl directly; player_ctrl is
    # part of the generic list already, so only the bench is needed here.
    'tb_player_emg_release' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_player_emg_release.v', 'rtl\player_ctrl.v')
    }
    'tb_emergency_alert_seg119' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_emergency_alert_seg119.v', 'rtl\emergency_alert.v')
    }
    'tb_fat32_file_reader' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_fat32_file_reader.v', 'rtl\storage\fat32_file_reader.v')
    }
    'tb_fire_wav_hq' = @{
        Mode  = '-g2012'
        Files = @('sim\tb_fire_wav_hq.v', 'rtl\audio\wav_voice_stream.v',
                  'rtl\audio\ulaw_codec.v')
    }
    # The scaler bench drives bmp_scaler directly, so it must carry the scaler
    # and the parser itself; those two are pulled into hdmi_mm_player.v through
    # `include and are therefore missing from the generic file list.
    'tb_bmp_scale_chain' = @{
        Mode  = '-g2005'
        Files = @('sim\tb_bmp_scale_chain.v', 'rtl\storage\bmp_scaler.v',
                  'rtl\storage\bmp_stream_parser.v', 'sim\check_stubs.v')
    }
}

$summary = New-Object System.Collections.Generic.List[string]

foreach ($tb in $tbs) {
    $tbFile = Join-Path $Proj ("sim\" + $tb + ".v")
    if (!(Test-Path $tbFile)) {
        $summary.Add(("{0,-26} MISSING" -f $tb))
        continue
    }
    $vvp = Join-Path $out ($tb + '.vvp')
    $log = Join-Path $out ($tb + '.log')
    $clog = Join-Path $out ($tb + '.compile.log')
    if ($special.ContainsKey($tb)) {
        $mode  = $special[$tb].Mode
        $files = @($special[$tb].Files | ForEach-Object { Join-Path $Proj $_ })
    } else {
        $mode  = '-g2005'
        $files = @($tbFile) + $common + $rtl
    }

    $co = & $iv $mode -I (Join-Path $Proj 'rtl') -s $tb -o $vvp @files 2>&1
    if ($LASTEXITCODE -ne 0) {
        # a few benches use SystemVerilog constructs; retry in that mode
        if ($mode -ne '-g2012') {
            $co = & $iv -g2012 -I (Join-Path $Proj 'rtl') -s $tb -o $vvp @files 2>&1
            if ($LASTEXITCODE -eq 0) { $co = @('note: compiled with -g2012') + $co }
        }
    }
    $co | Set-Content -Encoding ASCII $clog
    if ($LASTEXITCODE -ne 0) {
        $summary.Add(("{0,-26} COMPILE-FAIL" -f $tb))
        continue
    }

    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $tmpOut = Join-Path $out ($tb + '.out.tmp')
    $tmpErr = Join-Path $out ($tb + '.err.tmp')
    # benches that $fopen sim/build_adv01/... need the project root as cwd
    $args0 = @('"' + $vvp + '"')
    $p = Start-Process -FilePath $vv -ArgumentList $args0 -NoNewWindow -PassThru `
                       -WorkingDirectory $Proj `
                       -RedirectStandardOutput $tmpOut -RedirectStandardError $tmpErr
    $done = $p.WaitForExit($TimeoutSec * 1000)
    $sw.Stop()
    if (!$done) {
        try { $p.Kill() } catch { }
        $summary.Add(("{0,-26} TIMEOUT after {1:N0}s" -f $tb, $sw.Elapsed.TotalSeconds))
        continue
    }

    $txt = @()
    if (Test-Path $tmpOut) { $txt += Get-Content $tmpOut }
    if (Test-Path $tmpErr) { $txt += Get-Content $tmpErr }
    $txt | Set-Content -Encoding ASCII $log

    $res = ($txt | Select-String -Pattern 'RESULT:') | Select-Object -Last 1
    if ($null -ne $res) {
        $line = $res.Line.Trim()
    } else {
        $verdictLine = ($txt | Select-String -Pattern 'PASS|FAIL') | Select-Object -Last 1
        if ($null -ne $verdictLine) {
            $line = $verdictLine.Line.Trim()
        } else {
            $last = ($txt | Where-Object { $_.Trim().Length -gt 0 } | Select-Object -Last 1)
            $line = if ($null -ne $last) { 'last: ' + $last.Trim() } else { 'no output' }
        }
    }
    $verdict = if ($line -match 'PASS') { 'PASS  ' } elseif ($line -match 'FAIL') { 'FAIL  ' } else { 'UNKNOWN' }
    $summary.Add(("{0,-26} {1} {2}  ({3:N1}s)" -f $tb, $verdict, $line, $sw.Elapsed.TotalSeconds))
}

$summary | ForEach-Object { $_ }
$summary | Set-Content -Encoding ASCII (Join-Path $out 'summary.txt')
