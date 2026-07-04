# hitch report — joints

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `joints` |
| date | 2026-07-04T08:04:00.380Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 240 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.328 | 0.316 | 0.352 | 0.515 | 0.896 | 0.973 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |  1186 ( 98.8%) ████████████████████████████████████████
   0.5-1ms |    14 (  1.2%) 
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
|   1 |       1053 | 0.973 |
|   2 |        162 | 0.896 |
|   3 |       1052 | 0.796 |
|   4 |       1054 | 0.763 |
|   5 |        165 | 0.674 |

