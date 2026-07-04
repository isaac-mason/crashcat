# hitch report — body-churn

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `body-churn` |
| date | 2026-07-04T08:04:57.930Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 150 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |    max |
| ----: | ----: | ----: | ----: | ----: | -----: |
| 0.714 | 0.653 | 0.835 | 2.071 | 3.394 | 10.720 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     1 (  0.1%) 
   0.5-1ms |  1155 ( 96.3%) ████████████████████████████████████████
     1-2ms |    31 (  2.6%) █
     2-4ms |    12 (  1.0%) 
     4-8ms |     0 (  0.0%) 
    8-16ms |     1 (  0.1%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |     ms |
| --: | ---------: | -----: |
|   1 |        283 | 10.720 |
|   2 |        277 |  3.394 |
|   3 |        251 |  2.963 |
|   4 |        342 |  2.746 |
|   5 |         88 |  2.479 |

