# Which place strategies does this build accept?
import_device eagle_s20.db -package EG4S20NG88
foreach v {0 1 2 3 4 5} {
    if {[catch {set_param place pr_strategy $v} msg]} {
        puts "\[strategy\] $v -> REJECTED: $msg"
    } else {
        puts "\[strategy\] $v -> accepted"
    }
}
