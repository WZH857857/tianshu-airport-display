# Rebuild ADV01 with several placement seeds and stop at the first one whose
# routed timing closes.  The FAT32 state decode path is the critical path in
# this design and its slack swings by hundreds of picoseconds from seed to
# seed, so a failing seed is retried rather than accepted.
#
# Every build lands in hdmi_adv01_subtitle.bit; a copy per seed is kept so the
# best result can be picked afterwards.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$td  = 'E:\Anlogic\TD_6.2.1_Engineer_6.2.168.116\bin\td_commands_prompt.exe'
$seeds = @(5, 6, 7)
$results = @()

# The previous pass already produced these; they take part in the final pick.
$results += [pscustomobject]@{ seed = 2; setup_ns = -0.051; hold_ns = 0.014; bit = 'adv01_seed2.bit' }
$results += [pscustomobject]@{ seed = 3; setup_ns =  0.055; hold_ns = 0.011; bit = 'adv01_seed3.bit' }
$results += [pscustomobject]@{ seed = 4; setup_ns =  0.285; hold_ns = 0.003; bit = 'adv01_seed4.bit' }

foreach ($seed in $seeds) {
    Write-Host "=== ADV01 build with place seed $seed ==="
    $env:ADV01_SEED = "$seed"
    & $td 'scripts/manual_build_adv01_subtitle.tcl' 2>&1 |
        Select-String -Pattern "ADV01 SUBTITLE|USR-8108|USR-8159|Oops" |
        ForEach-Object { $_.Line }

    $phy = Get-Content 'hdmi_adv01_subtitle_phy.ts' -Raw
    $setup = [double]([regex]::Match($phy, 'SWNS:\s*(-?[\d.]+)ns').Groups[1].Value)
    $hold  = [double]([regex]::Match($phy, 'HWNS:\s*(-?[\d.]+)ns').Groups[1].Value)
    $copy  = "adv01_seed$seed.bit"
    Copy-Item 'hdmi_adv01_subtitle.bit' $copy -Force

    $results += [pscustomobject]@{ seed = $seed; setup_ns = $setup; hold_ns = $hold; bit = $copy }
    Write-Host ("seed {0}: setup {1} ns, hold {2} ns -> {3}" -f $seed, $setup, $hold, $copy)

    if ($setup -gt 0.15 -and $hold -gt 0.008) {
        Write-Host "seed $seed closes timing"
        break
    }
}

$results | Format-Table -AutoSize

# Keep the best routed result as the delivered bitstream.  Hold slack is the
# tighter of the two here, so only seeds with a usable hold window compete on
# setup slack.
$best = $results | Where-Object { $_.setup_ns -gt 0 -and $_.hold_ns -ge 0.008 } |
        Sort-Object -Property setup_ns -Descending | Select-Object -First 1
if (-not $best) {
    $best = $results | Where-Object { $_.setup_ns -gt 0 -and $_.hold_ns -gt 0 } |
            Sort-Object -Property hold_ns -Descending | Select-Object -First 1
}
if ($best) {
    Copy-Item $best.bit 'hdmi_adv01_subtitle.bit' -Force
    Write-Host ("best seed: {0} (setup {1} ns, hold {2} ns)" -f $best.seed, $best.setup_ns, $best.hold_ns)
} else {
    Write-Host "no seed closed timing"
    exit 1
}
