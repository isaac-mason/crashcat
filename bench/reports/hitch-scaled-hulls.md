# hitch report — scaled-hulls

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `scaled-hulls` |
| date | 2026-07-04T08:04:52.931Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 1.965 | 1.914 | 2.234 | 2.722 | 4.375 | 5.043 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     0 (  0.0%) 
   0.5-1ms |     0 (  0.0%) 
     1-2ms |   792 ( 66.0%) ████████████████████████████████████████
     2-4ms |   406 ( 33.8%) █████████████████████
     4-8ms |     2 (  0.2%) 
    8-16ms |     0 (  0.0%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |    ms |
| --: | ---------: | ----: |
|   1 |        412 | 5.043 |
|   2 |        413 | 4.375 |
|   3 |        411 | 3.953 |
|   4 |       1050 | 3.095 |
|   5 |       1071 | 2.973 |

