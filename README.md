# zig-pathfinder

A simple pathfinder for Zig.

This is a standalone version of the pathfinder I wrote for my
little custom game engine tailored to my needs.

Currently there's only the A* algorithm implemented.

Updates will most likely only happen if my game demands it.

It is not perfect and can always be improved but it works.

## Installation

```sh
zig fetch --save git+https://github.com/pgml/zig-pathfinder
```

Then add the following to `build.zig`:

```zig
const zpf = b.dependency("zig_pathfinder", .{});
exe.root_module.addImport("zig_pathfinder", zpf.module("zig_pathfinder"));
```

## Pathfinder Options

```zig
/// General pathfinder options
const Option = struct {
    /// Wether to use a separate thread to find a path for non-blocking 
    /// pathfinding
    use_separate_thead: bool = false,
};
```

## Path Options

```zig
/// Individual options for each path.
const PathOption = struct {
    /// The pathfinding algorithm to be used.
    algorithm: Algorithm = .astar,

    /// The movement mode for this path.
    movement: Movement = .orthogonal,

    /// The heuristic function that should be used to calculate the
    /// distance.
    heuristic: AStar.Heuristic = .manhattan,

    /// Whether to computed path should be in the centre of the tile.
    edge_centered: bool = false,
};

## A* heuristics
```zig
pub const Heuristic = enum {
    /// Best grid-based movement restricted to horizontal and vertical
    /// directions.
    manhattan,

    /// Best for grids where diagonal movement is allowed.
    euclidian,

    /// Best for 8-directional movement.
    /// (Horizontal, vertical and diagonal)
    chebyshev,
}
```

## Usage

```zig
const zpf = @import("zig_pathfinder");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer if (gpa.deinit() == .leak) std.process.exit(1);
    const alloc = gpa.allocator();

    const pathfinder: *zpf = try .init(alloc, .{});
    defer alloc.destroy(pathfinder);
    defer pathfinder.deinit();

    // Set the grid size for a 40x30 tilemap.
    pathfinder.setGridSize(40, 30);

    var non_walkable = [_]@Vector(2, f32){
        .{ 8, 15 },
        .{ 9, 15 },
        .{ 10, 15 },
        .{ 11, 15 },
    };

    try pathfinder.setNonWalkableTiles(&non_walkable);

    // Start the path at x: 10 and y: 10
    const start_pos: @Vector(2, f32) = .{ 10, 10 };

    // Set the target to x: 10 and y: 20
    const target_pos: @Vector(2, f32) = .{ 10, 20 };

    // Attempt to find a path
    try pathfinder.findPath(start_pos, target_pos, .{});

    // ...or with options
    try pathfinder.findPath(start_pos, target_pos, .{
        .edge_centered = true,
        .movement = .diagonal,
        .heuristic = .chebyshev,
    });

    if (pathfinder.getResult()) |path| {
        for (path) |node| {
            std.log.debug("path node: {}", .{node.pos});
        }
    }
    // result (10:15) and (11:15) are avoided:
    //
    // debug: path node: .{ .x = 10, .y = 11 }
    // debug: path node: .{ .x = 10, .y = 12 }
    // debug: path node: .{ .x = 10, .y = 13 }
    // debug: path node: .{ .x = 11, .y = 13 }
    // debug: path node: .{ .x = 12, .y = 13 }
    // debug: path node: .{ .x = 12, .y = 14 }
    // debug: path node: .{ .x = 12, .y = 15 }
    // debug: path node: .{ .x = 12, .y = 16 }
    // debug: path node: .{ .x = 12, .y = 17 }
    // debug: path node: .{ .x = 12, .y = 18 }
    // debug: path node: .{ .x = 12, .y = 19 }
    // debug: path node: .{ .x = 12, .y = 20 }
    // debug: path node: .{ .x = 11, .y = 20 }
    // debug: path node: .{ .x = 10, .y = 20 }
}
```
