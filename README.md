# RusTReach: A real time reachability tool written in Rust

# Running Example

```
cargo run --release -p bicycle_simple_exp -- 100 2.0 0.2 0.0 0.0 0.0 0.0 1.0 0.2666 true false
``` 

```
Usage: -- (milliseconds-runtime) (seconds-reachtime) (seconds-initial-stepsize) (x) (y) (linear velocity) (heading) (throttle control input) (heading control input) (load obstacles) (fixed step)
```

```
started
Argc: 12
runtime: 100 ms
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

interval list of obstacles: 
[1.935, 2.065], [1.935, 2.065]
[4.635, 4.765000000000001], [2.6350000000000002, 2.765]
[11.295, 11.424999999999999], [-1.525, -1.395]
[2.935, 3.065], [6.335, 6.465000000000001]
[-9.705, -9.575000000000001], [2.895, 3.025]

Opening file... with 5536 points
Quitting from runtime maxed out
[HyperRectangle (1.5279695153370967, 1.5296816505221065) (1.0324970245969902, 1.0337698649762477) (1.280207755680487, 1.2802858412469258) (1.1882642046422398, 1.1888472280190976)]

75ms: step_size = 0.000390625
iterations at quit: 10
Number of Rectangles: 5145

Number of Iterations: 10
done, result = safe
```