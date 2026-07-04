# hitch report — collapse

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `collapse` |
| date | 2026-07-04T08:04:55.749Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 0 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 |  p99.9 |    max |
| ----: | ----: | ----: | ----: | -----: | -----: |
| 0.220 | 0.006 | 0.451 | 4.089 | 15.928 | 32.471 |

## histogram

```
  0-0.25ms |  1058 ( 88.2%) ████████████████████████████████████████
0.25-0.5ms |    32 (  2.7%) █
   0.5-1ms |    56 (  4.7%) ██
     1-2ms |    34 (  2.8%) █
     2-4ms |     8 (  0.7%) 
     4-8ms |     5 (  0.4%) 
    8-16ms |     6 (  0.5%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     1 (  0.1%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |     ms |
| --: | ---------: | -----: |
|   1 |          0 | 32.471 |
|   2 |          3 | 15.928 |
|   3 |          1 | 15.599 |
|   4 |          4 | 11.681 |
|   5 |          2 |  9.323 |

