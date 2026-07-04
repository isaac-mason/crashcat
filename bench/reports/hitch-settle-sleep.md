# hitch report — settle-sleep

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `settle-sleep` |
| date | 2026-07-04T08:04:54.114Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.019 | 0.016 | 0.019 | 0.059 | 0.701 | 0.766 |

## histogram

```
  0-0.25ms |  1197 ( 99.8%) ████████████████████████████████████████
0.25-0.5ms |     1 (  0.1%) 
   0.5-1ms |     2 (  0.2%) 
     1-2ms |     0 (  0.0%) 
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
|   1 |        180 | 0.766 |
|   2 |        194 | 0.701 |
|   3 |        122 | 0.287 |
|   4 |        107 | 0.245 |
|   5 |        141 | 0.226 |

