# Sweep placement seeds for the ADV15 SW2 flip build and stop at the first
# seed that closes timing.  The design uses 97% of the EG4S20 slices, so the
# scaler adder chain in sys_clk lands a few picoseconds either side of the
# 20 ns period depending on the seed.  The critical path is inside
# bmp_scaler, not in the flip logic added by ADV15.
param(
    [string]   $Proj = 'E:\FPGA guo sai\jing jie yao qiu\hdmi_mm_player_advanced',
    [string]   $Td   = 'E:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe',
    [int[]]    $Seeds = @(2, 3, 4, 5, 6, 7)
)

Remove-Item Env:ADV15_STAGE -ErrorAction SilentlyContinue

# seed 1 (the ADV13/ADV14 seed) gives -146 ps setup with one endpoint and
# seed 0 gives -506 ps with six, so the sweep now starts at seed 2.
"seeds to try: $($Seeds -join ', ')"

$summary = @()

foreach ($seed in $Seeds) {
    $env:ADV15_SEED = "$seed"
    $qor = Join-Path $Proj 'hdmi_adv15_flip_route.qor'
    if (Test-Path -LiteralPath $qor) { Remove-Item -LiteralPath $qor -Force }
    $log = Join-Path $Proj "log\adv15_seed$seed.log"

    # td_commands_prompt.exe is an interactive shell: it stays alive at its
    # "td >" prompt once the tcl script is done.  Feeding it end-of-file from
    # NUL makes the process exit instead of hanging the sweep on its stdin.
    Push-Location $Proj
    & cmd /c "`"$Td`" `"scripts/manual_build_adv15_flip.tcl`" < NUL" *> $log
    Pop-Location

    if (!(Test-Path -LiteralPath $qor)) {
        "seed=$seed  no route qor, see $log"
        $summary += "seed=$seed  no route qor"
        continue
    }

    $txt   = Get-Content -LiteralPath $qor -Raw
    $setup = [regex]::Match($txt, 'Setup WNS\s*\|\s*(-?\d+)').Groups[1].Value
    $sep   = [regex]::Match($txt, 'violated S-EP\s*\|\s*(-?\d+)').Groups[1].Value
    $hold  = [regex]::Match($txt, 'Hold WNS\s*\|\s*(-?\d+)').Groups[1].Value
    $hep   = [regex]::Match($txt, 'violated H-EP\s*\|\s*(-?\d+)').Groups[1].Value
    $slice = [regex]::Match($txt, 'slices percentage\s*\|\s*([\d.]+)%').Groups[1].Value

    "seed={0}  setup={1}ps S-EP={2}  hold={3}ps H-EP={4}  slices={5}%" -f `
        $seed, $setup, $sep, $hold, $hep, $slice
    $summary += ("seed={0}  setup={1}ps S-EP={2}  hold={3}ps H-EP={4}  slices={5}%" -f `
        $seed, $setup, $sep, $hold, $hep, $slice)

    if (([int]$setup -gt 0) -and ([int]$sep -eq 0) -and
        ([int]$hold -gt 0) -and ([int]$hep -eq 0)) {
        Copy-Item -LiteralPath (Join-Path $Proj 'hdmi_adv15_flip.bit') `
                  -Destination (Join-Path $Proj "adv15_flip_seed${seed}_pass.bit") -Force
        "=== PASS at seed $seed ==="
        $summary += "=== PASS at seed $seed ==="
        break
    }
}

$summary | Set-Content -LiteralPath (Join-Path $Proj 'log\adv15_seed_sweep_summary.txt')
