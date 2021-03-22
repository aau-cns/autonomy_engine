---
title: AMAZE Autonomy
author: Christian Brommer
date: 19.03.2021
subtitle: Autonomy Version 0.0.1
---

Maintainer: Christian Brommer @chrbrommer

# Getting Started

**Work in progress**

The safety node a.k.a. watchdog, only starts if the Autonomy publishes a service request to determine if the system is ready for take-off. After this, the safety node will open two streams; the action service information which will communicate the latest and highest issue of the system which will determine the most likely action of the autonomy engine, and a system info with all 'delta' sensor informations.

Possible states for the action and info streams, are:

| Value | Description                   | Action | Info | Continue Mission |
| ----- | ----------------------------- | ------ | ---- | ---------------- |
| 1     | Nominal Condition             |        | x    | x                |
| 2     | Non-Critical Failure          |        | x    | x                |
| 4     | Inconvenient Failure          | x      | x    | x                |
| 8     | Severe Failure / Mission Hold | x      | x    |                  |
| 16    | Error / Mission Abort         | x      | x    |                  |



| Nominal Failure               | Description                                                  |
| ----------------------------- | ------------------------------------------------------------ |
| Nominal Condition             | The component is operating as intended                       |
| Non-Critical Failure          | The component has failed but is not system critical OR a frame rate is not as intended |
| Inconvenient Failure          | The component has failed and is not critical to the system but losing the data stream for recording is very inconvenient (e.g., the stereo camera). |
| Severe Failure / Mission Hold | A severe failure occurred, the safety node tries to restart the component, the mission should hold meanwhile. |
| Error / Mission Abort         | A severe failure could not be resolved, the system needs to land immediately |

