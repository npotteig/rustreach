# RusTReach: A real time reachability tool written in Rust

## Simple Reachability Bicycle Example

```shell
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

## Real-Time Reachability Control for Bicycle Model
```shell
cargo run --release -p bicycle_simple_ctrl
```

```
interval list of obstacles: 
[1.75, 2.25], [0.44999999999999996, 0.95]
[1.75, 2.25], [-0.95, -0.44999999999999996]

Goal 0: [4, 0]
The state at time 0 s is: 
 [0,0,0,1.5707963267948966] 

Goal 0 Reached [4, 4]
The state after 4.200000000000001 s is: 
 [3.8786686384028077,0.008984564077128533,2.00257028102642,-0.08368162638123001]
```

## Simple Reachability Quadcopter Example

A 12-DoF linear quadcopter model for a DJI F450.

state is `[x, y, z, phi, theta, psi, u, v, w, p, q, r]`

```shell
cargo run -p quadcopter_simple_exp
```

```
initial state: 
x[0]: 0
x[1]: 0
x[2]: 0
x[3]: 0
x[4]: 0
x[5]: 0
x[6]: 0
x[7]: 0
x[8]: 0
x[9]: 0
x[10]: 0
x[11]: 0

The state after 1.9999999999998126 s is: 
 
x[0]: 0.531600970293752
x[1]: 0.531600970293752
x[2]: 0
x[3]: 0.16261788617886716
x[4]: -0.16261788617886716
x[5]: 0
x[6]: 1.063414623512223
x[7]: 1.063414623512223
x[8]: 0
x[9]: 0.16261788617885475
x[10]: -0.16261788617885475
x[11]: 0
Number of States: 10002

Running reachability analysis

Number of Rectangles: 5147

Last Rectangle: 
[HyperRectangle (0, 0.5323329401409397) (0, 0.5323329401409397) (0, 0) (0, 0.16263351398297995) (-0.16263351398297995, 0) (0, 0) (0, 1.0640401142737297) (0, 1.0640401142737297) (0, 0) (0, 0.16260162601626088) (-0.16260162601626088, 0) (0, 0)]

Number of Iterations: 10
done, result = safe
```

## Real-time Reachability Control for Quadcopter Model
```shell
cargo run --release -p quadcopter_simple_ctrl
```
Perturbed inital velocity 0.5 m/s in the y direction
```
interval list of obstacles: 
[1.75, 2.25], [0.44999999999999996, 0.95]
[1.75, 2.25], [-0.95, -0.44999999999999996]

Goal 0: [4, 0, 0]
initial state: 
x[0]: 0
x[1]: 0
x[2]: 0
x[3]: 0
x[4]: 0
x[5]: 0
x[6]: 0
x[7]: 0.5
x[8]: 0
x[9]: 0
x[10]: 0
x[11]: 0

Goal 0 Reached [4, 0, 0]
The state after 3.600000000000002 s is: 
x[0]: 3.7921972780241258
x[1]: -0.02653440105393088
x[2]: 0
x[3]: 0.04193071113397394
x[4]: 0.5785894899326064
x[5]: -0.6296314572806609
x[6]: 1.1013285808653615
x[7]: -0.14827875303549107
x[8]: 0
x[9]: 0.026224409730078634
x[10]: -0.2628017573629675
x[11]: 1.0992535880219032
```