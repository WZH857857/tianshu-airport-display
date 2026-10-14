# Sweep placement seeds for the flight-board build and stop at the first one
# that closes timing.  The design sits above 95% of the EG4S20 slices, so a
# single unlucky seed can push the FAT32 adder chain in sys_clk a few
# picoseconds over the 20 ns period while every other seed passes.
param(
    [string]   $Proj = 'E:\FPGA guo sai\jing jie yao qiu\hdmi_mm_player_advanced',
    [string]   $Td   = 'E:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe',
    [int[]]    $Seeds = @(1, 0, 2, 4, 5, 6)
)

Remove-Item Env:ADV13_STAGE -ErrorAction SilentlyContinue

foreach ($seed in $Seeds) {
    $env:ADV13_SEED = "$seed"
    $qor = Join-Path $Proj 'hdmi_adv13_board_route.qor'
    if (Test-Path -LiteralPath $qor) { Remove-Item -LiteralPath $qor -Force }
    $log = Join-Path $env:TEMP "adv13_seed$seed.log"

    Push-Location $Proj
    & $Td 'scripts/manual_build_adv13_board.tcl' *> $log
    Pop-Location

    if (!(Test-Path -LiteralPath $qor)) {
        "seed=$seed  no route qor, see $log"
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

    if (([int]$setup -gt 0) -and ([int]$sep -eq 0) -and
        ([int]$hold -gt 0) -and ([int]$hep -eq 0)) {
        Copy-Item -LiteralPath (Join-Path $Proj 'hdmi_adv13_board.bit') `
                  -Destination (Join-Path $Proj "adv13_seed${seed}_pass.bit") -Force
        "=== PASS at seed $seed ==="
        break
    }
}
