# hitch report — mesh-field

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `mesh-field` |
| date | 2026-07-04T08:04:05.976Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.630 | 0.621 | 0.684 | 0.857 | 1.043 | 1.109 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     0 (  0.0%) 
   0.5-1ms |  1197 ( 99.8%) ████████████████████████████████████████
     1-2ms |     3 (  0.3%) 
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
|   1 |        273 | 1.109 |
|   2 |        934 | 1.043 |
|   3 |        239 | 1.042 |
|   4 |         31 | 0.984 |
|   5 |        140 | 0.974 |

