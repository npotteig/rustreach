# RusTReach: A real time reachability tool written in Rust

# Running Example

```
cargo run --release -- 100 2.0 0.2 0.0 0.0 0.0 0.0 1.0 0.2666 true false
``` 

```
Usage: -- (milliseconds-runtime) (seconds-reachtime) (seconds-initial-stepsize) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input) (load obstacles) (fixed step)
```

```
started
Argc: 12
runtime: 10 ms
x_0[0]: 0
x_0[1]: 0
x_0[2]: 0
x_0[3]: 0
u_0[0]: 1
u_0[1]: 0.2666


Quitting simulation: time 2.0000000000000013, step_size: 0.02
The state after 1.9800000000000013 s is: 
 [1.535948532647228,1.0223783392484023,1.282219550822527,1.1879440151461191] 

Number of States: 101

0ms: step_size = 0.1
iterations at quit: 1
Number of Rectangles: 27

Number of Iterations: 1
done, result = safe
```