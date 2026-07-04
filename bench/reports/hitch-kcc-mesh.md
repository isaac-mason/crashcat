# hitch report — kcc-mesh

_Per-step wall time over the post-warmup run. Raw timings, **GC pauses included** — a hitch IS the metric here, so nothing is filtered._

| field | value |
| --- | --- |
| scenario | `kcc-mesh` |
| date | 2026-07-04T08:04:02.544Z |
| git rev | `4b14620` (dirty) |
| node | v24.10.0 |
| cpu | Apple M1 Pro |
| warmup steps | 240 |
| timed steps | 1200 |

## step time (ms)

|  mean |   p50 |   p90 |   p99 | p99.9 |   max |
| ----: | ----: | ----: | ----: | ----: | ----: |
| 0.356 | 0.301 | 0.427 | 1.300 | 1.870 | 2.359 |

## histogram

```
  0-0.25ms |    86 (  7.2%) ███
0.25-0.5ms |  1021 ( 85.1%) ████████████████████████████████████████
   0.5-1ms |    58 (  4.8%) ██
     1-2ms |    34 (  2.8%) █
     2-4ms |     1 (  0.1%) 
     4-8ms |     0 (  0.0%) 
    8-16ms |     0 (  0.0%) 
   16-32ms |     0 (  0.0%) 
   32-64ms |     0 (  0.0%) 
    >=64ms |     0 (  0.0%) 
```

## 5 worst steps

|   # | step index |    ms |
| --: | ---------: | ----: |
|   1 |        292 | 2.359 |
|   2 |        185 | 1.870 |
|   3 |       1197 | 1.771 |
|   4 |        410 | 1.531 |
|   5 |        184 | 1.476 |

