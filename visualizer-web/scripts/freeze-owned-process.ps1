param([Parameter(Mandatory=$true)][int]$ServerProcessId,
      [Parameter(Mandatory=$true)][ValidateSet('plant','mcu')][string]$Kind)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
Add-Type -TypeDefinition @'
using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class OwnedFreeze {
  [DllImport("kernel32.dll", SetLastError=true)] static extern IntPtr OpenThread(uint access, bool inherit, uint id);
  [DllImport("kernel32.dll", SetLastError=true)] static extern uint SuspendThread(IntPtr handle);
  [DllImport("kernel32.dll", SetLastError=true)] static extern uint ResumeThread(IntPtr handle);
  [DllImport("kernel32.dll")] static extern bool CloseHandle(IntPtr handle);
  public static List<IntPtr> Freeze(int pid) {
    var handles = new List<IntPtr>();
    try {
      using (var process = Process.GetProcessById(pid)) {
        foreach (ProcessThread thread in process.Threads) {
          var handle = OpenThread(2, false, (uint)thread.Id);
          if (handle == IntPtr.Zero) throw new System.ComponentModel.Win32Exception();
          if (SuspendThread(handle) == uint.MaxValue) { CloseHandle(handle); throw new System.ComponentModel.Win32Exception(); }
          handles.Add(handle);
        }
      }
      if (handles.Count == 0) throw new Exception("No threads suspended");
      return handles;
    } catch { Resume(handles); throw; }
  }
  public static void Resume(List<IntPtr> handles) {
    foreach (var handle in handles) { ResumeThread(handle); CloseHandle(handle); }
  }
}
'@
$all = @(Get-CimInstance Win32_Process)
$server = $all | Where-Object ProcessId -EQ $ServerProcessId
if (!$server -or $server.Name -ne 'node.exe' -or $server.CommandLine -notmatch 'server\.ts') { throw 'Not the explicitly spawned test server' }
$ownedIds = [System.Collections.Generic.HashSet[int]]::new()
$null = $ownedIds.Add($ServerProcessId)
do {
  $changed = $false
  foreach ($item in $all) {
    if ($ownedIds.Contains([int]$item.ParentProcessId) -and $ownedIds.Add([int]$item.ProcessId)) { $changed = $true }
  }
} while ($changed)
$plants = @($all | Where-Object { $ownedIds.Contains([int]$_.ProcessId) -and $_.Name -eq 'plant-bridge.exe' })
if ($plants.Count -ne 1) { throw "Expected one owned real plant; got $($plants.Count)" }
$target = if ($Kind -eq 'plant') { $plants[0] } else {
  $all | Where-Object ProcessId -EQ $plants[0].ParentProcessId
}
if (!$ownedIds.Contains([int]$target.ProcessId) -or $target.ProcessId -eq $ServerProcessId) { throw 'Target outside owned descendant tree' }
if ($Kind -eq 'mcu' -and ($target.Name -ne 'node.exe' -or $target.CommandLine -notmatch 'web-bridge\.ts')) { throw 'Target is not actual MCU bridge parent of plant' }
$current = Get-CimInstance Win32_Process -Filter "ProcessId = $($target.ProcessId)"
if ($current.CreationDate -ne $target.CreationDate) { throw 'Target process identity changed' }
$handles = $null
try {
  $handles = [OwnedFreeze]::Freeze([int]$target.ProcessId)
  $children = @($all | Where-Object { $ownedIds.Contains([int]$_.ProcessId) -and $_.ProcessId -ne $ServerProcessId -and ($_.CommandLine -match 'web-bridge\.ts' -or $_.Name -eq 'plant-bridge.exe') } | Select-Object ProcessId,ParentProcessId,CreationDate,Name)
  @{ target = [int]$target.ProcessId; kind = $Kind; threads = $handles.Count; children = $children } | ConvertTo-Json -Compress -Depth 4
  # Parent sends a newline in finally, even when assertions fail. A dead parent
  # closes stdin. Resume exactly the increments made by this helper, never others.
  $null = [Console]::ReadLine()
} finally {
  if ($null -ne $handles) { [OwnedFreeze]::Resume($handles) }
}
