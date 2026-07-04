# hitch report — projectiles-terrain

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `projectiles-terrain` |
| date | 2026-07-04T08:04:08.082Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 240 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.340 | 0.306 | 0.481 | 0.773 | 1.002 | 1.069 |

## histogram

```
  0-0.25ms |    79 (  6.6%) ███
0.25-0.5ms |  1023 ( 85.3%) ████████████████████████████████████████
   0.5-1ms |    96 (  8.0%) ████
     1-2ms |     2 (  0.2%) 
     2-4ms |     0 (  0.0%) 
     4-8ms |     0 (  0.0%) 
    8-16ms |     0 (  0.0%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |    ms |
| --: | ---------: | ----: |
|   1 |        126 | 1.069 |
|   2 |        124 | 1.002 |
|   3 |        530 | 0.912 |
|   4 |        201 | 0.868 |
|   5 |        334 | 0.841 |

