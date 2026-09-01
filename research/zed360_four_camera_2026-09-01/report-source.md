# Four ZED 2i / two Windows hosts: ZED360 and BODY_38 research source

Research date: 2026-09-01  
Rig: ZED 2i `39504762`, `34760587` on laptop `192.168.50.11`; ZED 2i
`33773329`, `31571870` on main PC `192.168.50.10`; wired 2.5 GbE; ZED SDK
5.4.1 on both hosts.

## Decision summary

1. Try ZED360 once with an official-sample-aligned **hybrid** configuration:
   the main PC's two USB cameras use `INTRA_PROCESS`; only the laptop's two
   cameras publish through `LOCAL_NETWORK`. This minimizes network publishers
   and lets ZED360 own the two local devices.
2. Calibrate ZED360 with `BODY_18`, tracking off, fitting off, MEDIUM,
   HD720@15 and NEURAL LIGHT. Use one slowly walking person in the entire
   overlapping volume.
3. If ZED360 prints `WRONG BODY FORMAT` although both laptop publishers report
   `received_format=0`, stop after one clean unplug/replug retry. Multiple 2026
   reports reproduce this error with identical publisher formats, so repeated
   format guessing is not an evidence-based recovery path.
4. Use the application-level BODY_38 path for today's production fallback.
   It timestamps packets on arrival at the main PC, records simultaneous
   four-view skeletons, estimates rigid camera-to-world extrinsics, robustly
   fuses per-joint measurements, and retains the existing
   `zed_body38_live/v1` contract for G1 retargeting.

## Evidence matrix

| Question | Strongest evidence | Finding | Confidence / gap |
|---|---|---|---|
| What does ZED360 calibrate? | [Official ZED360 documentation](https://docs.stereolabs.com/docs/development/zed-tools/zed-360) | It aligns body keypoints in a common WORLD frame. Initial camera icons can overlap; IMU supplies rotation before body-based optimization. | High. |
| How must the operator move? | [Official ZED360 documentation](https://docs.stereolabs.com/docs/development/zed-tools/zed-360) | Minimal FOV overlap, exactly one visible person, walk slowly across the complete volume; optimization runs roughly every ten seconds. | High. |
| Can local and network cameras coexist in one config? | [Official Stereolabs Python multi-camera sample](https://github.com/stereolabs/zed-sdk/blob/master/body%20tracking/multi-camera/python/fused_cameras.py) | The sample reads one Fusion config, opens non-network entries locally and treats `LOCAL_NETWORK` entries as already-running publishers. | High for Fusion API. The current ZED360 UI docs separately describe local and network workflows, so hybrid UI behavior is less explicitly documented. |
| What belongs in the config? | [Official Fusion documentation](https://docs.stereolabs.com/docs/development/zed-sdk/modules/fusion) | Per-camera serial/input, `INTRA_PROCESS` or `LOCAL_NETWORK` communication details, and camera world rotation/translation. | High. |
| Do distributed hosts require synchronized time? | [Official multi-camera setup](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera) | USB has no hardware trigger; network hosts use system clocks and Stereolabs recommends PTP. | High. Official PTP recipes are Linux-oriented; this two-Windows test must use a common Windows time source and tolerate more jitter, or use main-PC arrival timestamps in the fallback. |
| Are two ZED 2i cameras per host safe? | [Official multi-camera setup](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera) plus supplied diagnostics | USB controller bandwidth is the limiting resource; saturation causes tearing/corruption/disconnects. All four supplied per-camera diagnostics say USB 3 and bandwidth OK. | Medium-high. Per-camera diagnostics do not prove simultaneous dual-camera load, so both cameras must remain stable together at HD720@15 before calibration is accepted. |
| Recommended sender settings? | [Stereolabs support guidance](https://community.stereolabs.com/t/360-fusion-network-workflow/4715) and [official sample](https://github.com/stereolabs/zed-sdk/blob/master/body%20tracking/multi-camera/python/fused_cameras.py) | BODY_18 for calibration, tracking off, fitting off, positional tracking on, stable FPS (15 if needed); MEDIUM is support's recommended model. | Medium-high: support forum plus official sample, not a normative API constraint. |
| Is `WRONG BODY FORMAT` necessarily user misconfiguration? | [2026 Stereolabs forum reproductions](https://community.stereolabs.com/t/zed360-crashing-and-not-showing-skeletal-data/10694) and [related network Fusion report](https://community.stereolabs.com/t/framerate-issue-when-using-the-fusion-network-workflow/10593) | Several users report the error with identical formats and valid raw subscriber data on SDK 5.1/5.2. The issue was forwarded for reproduction; no definitive published fix is visible in those threads. | Medium. Forum evidence is not a formal bug notice, but it closely matches this rig's console and direct-probe results. |
| What changed in SDK 5.4? | [Official release notes](https://github.com/stereolabs/zed-sdk/releases/tag/5.4.0) | Streaming receiver restart/thread safety and Windows USB reopen/enumeration reliability were improved. | High for those fixes. The notes do not claim a ZED360 `WRONG BODY FORMAT` fix. |
| Does BODY_38 matter downstream? | [Official body tracking documentation](https://www.stereolabs.com/docs/development/zed-sdk/modules/body-tracking/using-the-api) and repository packet contract | BODY_38 carries the joints and local joint/root orientation data used by this project's G1 retargeting path. | High. ZED360 calibration can use BODY_18 because the exported extrinsics, not the 18-joint packet itself, are the reusable result. |

## Supplied diagnostic evidence

- Main PC reports `PC_1` and `PC_2`: ZED SDK 5.4.1, ZED 2i, USB mode 3,
  USB bandwidth OK, no camera-test error. Device paths are distinct (`/24`,
  `/20`) and three Intel xHCI controllers are listed.
- Laptop reports `Laptop_1` and `Laptop_2`: ZED SDK 5.4.1, ZED 2i, USB mode
  3, USB bandwidth OK, no camera-test error. Device paths are distinct (`/18`,
  `/22`) and two xHCI controllers are listed.
- These are necessary but not sufficient checks: each report tests one camera,
  so today's two-camera simultaneous publisher run remains the acceptance test.
- Earlier direct tests on this exact rig showed four publishers near 15 FPS and
  valid bodies, but ZED360 and a direct `sl.Fusion` probe both returned
  `WRONG BODY FORMAT` even after every sender reported the same format. That
  evidence raises the probability of a Fusion network compatibility defect.

## Contradictions and unresolved gaps

- Current ZED360 documentation describes network operation through ZED Hub and
  a dedicated subscriber host, while the installed Windows UI exposes a manual
  edge-local-network sender form. The official Fusion config/sample still
  supports `LOCAL_NETWORK`, but the manual ZED360 UI path is not documented in
  equivalent detail.
- The official sample is BODY_18. The Fusion API can operate with BODY_38, but
  ZED360 calibration is deliberately kept at BODY_18 to maximize compatibility.
- Stereolabs' precise distributed-host recommendation is PTP. No official
  Windows PTP setup for this exact direct-cable topology was found. The custom
  fallback therefore synchronizes by the main PC's packet-arrival clock and
  exposes arrival spread; this is robust for low-jitter 2.5 GbE but is not
  hardware frame synchronization.
- The previously generated extrinsics were numerically good, but the user
  confirmed that the physical camera positions changed on 2026-09-01. They are
  archived as stale evidence and a fresh record/calibration is mandatory.

## Acceptance thresholds used by this project

- Network: 8/8 successful 1400-byte pings in each direction; preferred average
  under 5 ms; no Wi-Fi path for camera packets.
- Sources: all four sustain 14.5-15.5 BODY frames/s for at least 60 seconds;
  zero SDK corrupted-frame errors; each sees one operator in the overlap.
- ZED360: four cameras present, non-zero detection ratios, camera models
  separate after optimization, fused skeleton visible, saved configuration.
- Custom calibration: four simultaneous BODY_38 sources; target RMS <= 0.08 m
  and p95 <= 0.12 m for every non-reference camera.
- Custom live fusion: acceptance test uses `MinimumSources=4`; `fusion_cikis`
  must grow continuously and `gecersiz` must remain zero. Runtime may later use
  2 or 3 minimum sources for occlusion resilience.

## Source quality note

Official Stereolabs documentation and the Stereolabs-owned SDK sample are the
primary basis. Forum posts are used only to interpret the observed failure and
are labeled as support/community evidence. No product listing, third-party
blog, or generative summary is used for a technical requirement.
