# 1. The SLAM Problem

Localization (Module 02): given **map** m, compute p(**x**t | m, **z₁:t**)  
Mapping: given **poses** **x₁:t**, compute map m  
**SLAM**: compute p(**x₁:t**, m | **z₁:t**, **u₁:t**) simultaneously

SLAM is hard because:
- Localization needs a map
- Mapping needs accurate poses
- Both are uncertain simultaneously

```
Robot moves → uncertainty in pose grows
Robot sees landmark → pose uncertainty collapses
But landmark position is also uncertain
Seeing a known landmark again (loop closure) collapses BOTH pose and landmark uncertainty
```
