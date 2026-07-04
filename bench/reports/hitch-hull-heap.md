# hitch report — hull-heap

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `hull-heap` |
| date | 2026-07-04T08:03:58.626Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |    p99 |  p99.9 |    max |
| ----: | ----: | ----: | -----: | -----: | -----: |
| 5.012 | 4.739 | 5.762 | 11.383 | 21.165 | 39.327 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     0 (  0.0%) 
   0.5-1ms |     0 (  0.0%) 
     1-2ms |     0 (  0.0%) 
     2-4ms |    41 (  3.4%) █
     4-8ms |  1129 ( 94.1%) ████████████████████████████████████████
    8-16ms |    27 (  2.3%) █
   16-32ms |     2 (  0.2%) 
   32-64ms |     1 (  0.1%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |     ms |
| --: | ---------: | -----: |
|   1 |        830 | 39.327 |
|   2 |        789 | 21.165 |
|   3 |        833 | 19.242 |
|   4 |        895 | 15.399 |
|   5 |         10 | 15.012 |

