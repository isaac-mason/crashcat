# hitch report — pyramid

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `pyramid` |
| date | 2026-07-04T08:04:42.112Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 300 |
| timed steps | 1200 |

## step time (ms)

|   mean |    p50 |    p90 |    p99 |  p99.9 |    max |
| -----: | -----: | -----: | -----: | -----: | -----: |
| 12.166 | 11.922 | 12.827 | 15.106 | 17.443 | 18.612 |

## histogram

```
  0-0.25ms |     0 (  0.0%) 
0.25-0.5ms |     0 (  0.0%) 
   0.5-1ms |     0 (  0.0%) 
     1-2ms |     0 (  0.0%) 
     2-4ms |     0 (  0.0%) 
     4-8ms |     0 (  0.0%) 
    8-16ms |  1196 ( 99.7%) ████████████████████████████████████████
   16-32ms |     4 (  0.3%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |     ms |
| --: | ---------: | -----: |
|   1 |        604 | 18.612 |
|   2 |        256 | 17.443 |
|   3 |         93 | 17.289 |
|   4 |        419 | 16.798 |
|   5 |        606 | 15.865 |

