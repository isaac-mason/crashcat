# hitch report — raycasts

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `raycasts` |
| date | 2026-07-04T08:04:44.468Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 300 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.533 | 0.534 | 0.637 | 0.813 | 0.963 | 1.004 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |   358 ( 29.8%) █████████████████
   0.5-1ms |   841 ( 70.1%) ████████████████████████████████████████
     1-2ms |     1 (  0.1%) 
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
|   1 |        555 | 1.004 |
|   2 |        547 | 0.963 |
|   3 |        526 | 0.936 |
|   4 |        180 | 0.923 |
|   5 |        199 | 0.905 |

