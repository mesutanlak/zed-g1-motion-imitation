param(
    [Parameter(Mandatory = $true)]
    [string]$TitleContains,
    [ValidateRange(1, 60)]
    [int]$TimeoutSeconds = 15
)

$ErrorActionPreference = "SilentlyContinue"

Add-Type @"
using System;
using System.Text;
using System.Runtime.InteropServices;

public static class WslgWindowFocus {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);

    [DllImport("user32.dll")]
    public static extern bool EnumWindows(EnumWindowsProc callback, IntPtr lParam);

    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder text, int count);

    [DllImport("user32.dll")]
    public static extern bool IsWindowVisible(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool IsIconic(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool ShowWindowAsync(IntPtr hWnd, int command);

    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);

    [DllImport("user32.dll")]
    public static extern bool BringWindowToTop(IntPtr hWnd);
}
"@

$deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
while ([DateTime]::UtcNow -lt $deadline) {
    $matchedWindow = [IntPtr]::Zero
    [WslgWindowFocus]::EnumWindows({
        param([IntPtr]$handle, [IntPtr]$parameter)
        if (-not [WslgWindowFocus]::IsWindowVisible($handle)) {
            return $true
        }
        $title = [Text.StringBuilder]::new(512)
        [void][WslgWindowFocus]::GetWindowText($handle, $title, $title.Capacity)
        if ($title.ToString().Contains($TitleContains)) {
            $script:matchedWindow = $handle
            return $false
        }
        return $true
    }, [IntPtr]::Zero) | Out-Null

    if ($matchedWindow -ne [IntPtr]::Zero) {
        # SW_RESTORE also handles a window that WSLg created minimized.
        [void][WslgWindowFocus]::ShowWindowAsync($matchedWindow, 9)
        [void][WslgWindowFocus]::BringWindowToTop($matchedWindow)
        [void][WslgWindowFocus]::SetForegroundWindow($matchedWindow)
        exit 0
    }
    Start-Sleep -Milliseconds 200
}

exit 1
