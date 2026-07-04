# hitch report — cube-heap

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `cube-heap` |
| date | 2026-07-04T08:03:38.564Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 2.443 | 2.368 | 2.830 | 3.535 | 4.719 | 4.950 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     0 (  0.0%) 
   0.5-1ms |     0 (  0.0%) 
     1-2ms |     7 (  0.6%) 
     2-4ms |  1190 ( 99.2%) ████████████████████████████████████████
     4-8ms |     3 (  0.3%) 
    8-16ms |     0 (  0.0%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |    ms |
| --: | ---------: | ----: |
|   1 |        432 | 4.950 |
|   2 |        101 | 4.719 |
|   3 |        778 | 4.433 |
|   4 |        815 | 3.928 |
|   5 |        788 | 3.915 |

