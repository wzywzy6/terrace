#requires -Version 5.1
<#
.SYNOPSIS
    独立于 serialplot 的自检工具：直接读串口并按 sp::Plotter 的协议解帧。

.DESCRIPTION
    帧格式（与 sp_middleware/io/plotter 一致）：
        AA BB | size(1字节, 只算后面 payload 的字节数) | float32 小端 x N (size = 4N)

    用途：区分「板子没发 / 发错口 / 波特率不对 / 上位机没解析」这几种情况。

.PARAMETER Port
    串口号，例如 COM9。

.PARAMETER Baud
    默认 921600，和 Src/usart.c 里 huart1 的配置一致。

.PARAMETER Seconds
    抓取时长，默认 5 秒。

.PARAMETER SelfTest
    不打开串口，用合成数据验证解析逻辑。

.EXAMPLE
    pwsh -ExecutionPolicy Bypass -File tools\serial_plot_check.ps1 -Port COM9

.EXAMPLE
    pwsh -ExecutionPolicy Bypass -File tools\serial_plot_check.ps1 -SelfTest
#>
param(
    [string]$Port,
    [int]$Baud = 921600,
    [double]$Seconds = 5,
    [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'

function ConvertFrom-PlotBytes {
    param([byte[]]$Bytes)

    $frames = New-Object System.Collections.Generic.List[object]
    $skipped = 0
    $i = 0
    while ($i + 3 -le $Bytes.Length) {
        if ($Bytes[$i] -eq 0xAA -and $Bytes[$i + 1] -eq 0xBB) {
            $size = [int]$Bytes[$i + 2]
            $total = 3 + $size
            if ($size -gt 0 -and ($size % 4) -eq 0 -and ($i + $total) -le $Bytes.Length) {
                $values = New-Object 'System.Collections.Generic.List[float]'
                for ($k = 0; $k -lt $size; $k += 4) {
                    $values.Add([BitConverter]::ToSingle($Bytes, $i + 3 + $k))
                }
                $frames.Add($values.ToArray())
                $i += $total
                continue
            }
        }
        $skipped++
        $i++
    }

    [pscustomobject]@{
        Frames  = $frames
        Skipped = $skipped
        Bytes   = $Bytes.Length
    }
}

function New-PlotFrame {
    param([float[]]$Values)
    $ms = New-Object System.IO.MemoryStream
    $bw = New-Object System.IO.BinaryWriter($ms)
    try {
        $bw.Write([byte]0xAA)
        $bw.Write([byte]0xBB)
        $bw.Write([byte]($Values.Length * 4))
        foreach ($v in $Values) { $bw.Write([single]$v) }
        $bw.Flush()
        return $ms.ToArray()
    }
    finally {
        $bw.Dispose()
        $ms.Dispose()
    }
}

function Write-Result {
    param($Report, [double]$Seconds)

    Write-Host ("收到字节数 : {0}" -f $Report.Bytes) -ForegroundColor Cyan
    Write-Host ("解析出帧数 : {0}" -f $Report.Frames.Count) -ForegroundColor Cyan
    if ($Report.Bytes -gt 0 -and $Seconds -gt 0) {
        Write-Host ("平均帧率   : {0:N0} Hz" -f ($Report.Frames.Count / $Seconds)) -ForegroundColor Cyan
    }

    if ($Report.Frames.Count -gt 0) {
        Write-Host ("通道数     : {0}" -f $Report.Frames[$Report.Frames.Count - 1].Length) -ForegroundColor Green
        $tail = [Math]::Min(5, $Report.Frames.Count)
        Write-Host "最后 $tail 帧:" -ForegroundColor Green
        for ($n = $Report.Frames.Count - $tail; $n -lt $Report.Frames.Count; $n++) {
            $vals = ($Report.Frames[$n] | ForEach-Object { '{0,10:N5}' -f $_ }) -join ''
            Write-Host ("  #{0,-6} {1}" -f $n, $vals)
        }
        Write-Host "链路 OK：板子确实在按 AA BB + size + float32 发数据。" -ForegroundColor Green
        return
    }

    if ($Report.Bytes -eq 0) {
        Write-Host ""
        Write-Host "一个字节都没收到，按顺序检查：" -ForegroundColor Yellow
        Write-Host "  1. 串口号是不是当下插着的那个（设备管理器里拔插确认）"
        Write-Host "  2. USB-TTL 的 RX 接板子 UART1 的 TX(PA9)，GND 必须共地"
        Write-Host "  3. 板子是否在跑（程序没停/没被调试器 halt）"
        return
    }

    Write-Host ""
    Write-Host "收到字节但解不出帧，按顺序检查：" -ForegroundColor Yellow
    Write-Host "  1. 波特率不是 921600（上位机/固件两端要对齐）"
    Write-Host "  2. 接的是 UART2(USART6, GY56@9600) 或别的串口，不是 UART1"
    Write-Host "  3. TX/RX 没交叉（两边 TX 对 TX）"
}

if ($SelfTest) {
    Write-Host "=== SelfTest: 用 3 帧合成数据验证解析 ===" -ForegroundColor Cyan
    $stream = New-Object System.Collections.Generic.List[byte]
    $stream.AddRange([byte[]](New-PlotFrame -Values @(0.292, 0.010, 0.282)))
    $stream.AddRange([byte[]](0x11, 0x22))
    $stream.AddRange([byte[]](New-PlotFrame -Values @(0.300, 0.020, 0.280)))
    $stream.AddRange([byte[]](New-PlotFrame -Values @(0.310, 0.030, 0.280)))
    $report = ConvertFrom-PlotBytes -Bytes $stream.ToArray()
    Write-Result -Report $report -Seconds 0
    if ($report.Frames.Count -ne 3) { Write-Host "SelfTest FAILED" -ForegroundColor Red; exit 1 }
    Write-Host "SelfTest PASSED" -ForegroundColor Green
    exit 0
}

if (-not $Port) {
    Write-Host "可用串口：" -ForegroundColor Yellow
    [System.IO.Ports.SerialPort]::GetPortNames() | ForEach-Object { Write-Host "  $_" }
    Write-Host "用法: pwsh -ExecutionPolicy Bypass -File tools\serial_plot_check.ps1 -Port COM9 -Baud 921600 -Seconds 5"
    exit 1
}

Write-Host ("打开 {0} @ {1} 8N1，抓取 {2} 秒 ..." -f $Port, $Baud, $Seconds) -ForegroundColor Cyan

$serial = New-Object System.IO.Ports.SerialPort($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 1000
try {
    $serial.Open()
}
catch {
    Write-Host ("打开 {0} 失败: {1}" -f $Port, $_.Exception.Message) -ForegroundColor Red
    Write-Host "串口被 serialplot / 串口助手占用了？同一时间只能一个程序打开。" -ForegroundColor Yellow
    exit 1
}

$buffer = New-Object System.Collections.Generic.List[byte]
try {
    $deadline = [datetime]::UtcNow.AddSeconds($Seconds)
    while ([datetime]::UtcNow -lt $deadline) {
        $available = $serial.BytesToRead
        if ($available -gt 0) {
            $chunk = New-Object byte[] $available
            $read = $serial.Read($chunk, 0, $available)
            for ($i = 0; $i -lt $read; $i++) { $buffer.Add($chunk[$i]) }
        }
        else {
            Start-Sleep -Milliseconds 20
        }
    }
}
finally {
    $serial.Close()
    $serial.Dispose()
}

$report = ConvertFrom-PlotBytes -Bytes $buffer.ToArray()
Write-Result -Report $report -Seconds $Seconds
