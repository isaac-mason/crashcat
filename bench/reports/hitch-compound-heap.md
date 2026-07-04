# hitch report — compound-heap

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `compound-heap` |
| date | 2026-07-04T08:05:02.040Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.439 | 0.426 | 0.546 | 0.670 | 1.353 | 1.780 |

## histogram

```
  0-0.25ms |     8 (  0.7%) 
0.25-0.5ms |   906 ( 75.5%) ████████████████████████████████████████
   0.5-1ms |   282 ( 23.5%) ████████████
     1-2ms |     4 (  0.3%) 
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
|   1 |        508 | 1.780 |
|   2 |        121 | 1.353 |
|   3 |        509 | 1.173 |
|   4 |        152 | 1.085 |
|   5 |        153 | 0.751 |

