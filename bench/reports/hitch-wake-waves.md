# hitch report — wake-waves

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `wake-waves` |
| date | 2026-07-04T08:04:59.529Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 600 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.347 | 0.307 | 0.465 | 1.126 | 1.243 | 1.305 |

## histogram

```
  0-0.25ms |    99 (  8.3%) ████
0.25-0.5ms |  1003 ( 83.6%) ████████████████████████████████████████
   0.5-1ms |    77 (  6.4%) ███
     1-2ms |    21 (  1.8%) █
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
|   1 |        205 | 1.305 |
|   2 |        206 | 1.243 |
|   3 |        199 | 1.230 |
|   4 |        202 | 1.172 |
|   5 |        204 | 1.166 |

