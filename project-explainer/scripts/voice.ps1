$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$project_dir = Split-Path $PSScriptRoot -Parent
$clips = Get-Content -LiteralPath (Join-Path $project_dir 'tts-input.json') -Raw -Encoding utf8 | ConvertFrom-Json
$synth = New-Object System.Speech.Synthesis.SpeechSynthesizer
$synth.SelectVoice('Microsoft Huihui Desktop')
$synth.Rate = 3
$format = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(22050, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
foreach ($clip in $clips)
{
    $audio_path = Join-Path $project_dir ('.voice-cache/clip-{0:D4}.wav' -f [int]$clip.id)
    $synth.SetOutputToWaveFile($audio_path, $format)
    $synth.Speak([string]$clip.text)
    $synth.SetOutputToNull()
    if ([int]$clip.id % 40 -eq 0)
    {
        Write-Output ('Speech clips: {0}/{1}' -f $clip.id, $clips.Count)
    }
}
$synth.Dispose()
Write-Output 'Chinese narration generated.'
