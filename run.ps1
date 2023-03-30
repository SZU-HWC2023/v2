$maps = "1.txt", "2.txt", "3.txt", "4.txt"
$total_score = 0

foreach ($map in $maps) {
    $output = ../Robot "./build/main" -m "../maps/$map" -f 2> "../v2/code/err.txt"
    $score = ($output | ConvertFrom-Json).score
    $total_score += $score
    Write-Host "Map $map score: $score"
}

Write-Host "Total score: $total_score"
