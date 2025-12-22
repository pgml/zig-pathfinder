pub const Pathfinder = @This();

const std = @import("std");
const AStar = @import("astar.zig");

/// The main pathfinder arena.
arena: std.heap.ArenaAllocator,

/// The main pathfinder allocator.
alloc: std.mem.Allocator,

/// The arena for temporary thread data.
work_arena: std.heap.ArenaAllocator,

/// The allocator that should be used for data allocated in the thread.
work_alloc: std.mem.Allocator,

/// The thread that is used for threaded pathfinding.
///
/// This has no effect if `Option.use_separate_thread` is false.
thread: ?std.Thread = null,

/// This has no effect if `Option.use_separate_thread` is false.
mutex: std.Thread.Mutex = .{},

/// This has no effect if `Option.use_separate_thread` is false.
cond: std.Thread.Condition = .{},

/// Indicates whether the thread worker should attempt to find a path.
/// If false the thread does nothing until `findPath()` is called.
///
/// This has no effect if `Option.use_separate_thread` is false.
working: bool = false,

/// Indicates whether the thread should quit.
///
/// This has no effect if `Option.use_separate_thread` is false.
quit: bool = false,

/// The current path object
path: Path,

/// The pathfinding result
result: ?[]PathNode = null,

opts: Option,

/// The width of the grid.
/// This is important to keep the pathfinder from accidently
/// trying to
grid_width: f32 = 0,

grid_height: f32 = 0,

/// A list of tiles/cells that should be excluded from pathfinding.
non_walkable_tiles: std.ArrayList(Vector2) = .empty,

/// Pathfinding algoritms.
const Algorithm = enum {
    astar,
};

const Movement = enum {
    /// Restrict movement to be horizontally and vertically only.
    orthogonal,

    /// Allow diagonal movement.
    diagonal,
};

/// General pathfinder options
const Option = struct {
    /// Wether to use a separate thread to find a path.
    /// This should probably true if pathfinding should be non-blocking.
    use_separate_thead: bool = false,
};

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

/// Path holds information about the found path.
/// It provides functions to update the path.
pub const Path = struct {
    /// The start position of the path.
    start_node: *PathNode = undefined,

    /// The path destination
    target_node: *PathNode = undefined,

    /// The nodes of the path to reach target position from start position.
    nodes: ?[]PathNode = null,

    /// Whether a path has been found.
    found: bool = false,

    opts: PathOption = .{},

    /// A pointer to the the list that holds the tiles that should be
    /// excluded from pathfinding.
    non_walkable_tiles: *std.ArrayList(Vector2),

    pub fn update(self: *Path, nodes: []PathNode) void {
        self.nodes = nodes;
        self.found = true;
    }

    fn reset(self: *Path) void {
        self.start_node = undefined;
        self.target_node = undefined;
        self.nodes = null;
        self.found = false;
    }
};

/// A single node of a path.
pub const PathNode = struct {
    /// This must be the arena allocator that is used to find the path
    /// so that we can easily clean up all allocations used for the
    /// pathfinding process.
    arena_alloc: std.mem.Allocator,

    /// The position of the node on the grid.
    pos: Vector2,

    /// The g cost for the a* algorithm
    g: f32 = 0,

    /// The h cost for the a* algorithm
    h: f32 = 0,

    /// The neighbouring grid cells of the current node
    neighbours: std.ArrayList(*PathNode) = .empty,

    /// connection represents the neighbouring node that is used
    /// to builds the path.
    connection: ?*PathNode = null,

    pub fn init(
        position: Vector2,
        arena_alloc: std.mem.Allocator,
    ) !*PathNode {
        const self = try arena_alloc.create(PathNode);
        self.* = .{ .arena_alloc = arena_alloc, .pos = position };
        return self;
    }

    /// Sets the g-cost of the A* algorithm for this node.
    pub fn setG(self: *PathNode, g: f32) void {
        self.g = g;
    }

    /// Sets the h-cost of the A* algorithm for this node.
    pub fn setH(self: *PathNode, h: f32) void {
        self.h = h;
    }

    /// Returns the A* f-cost.
    pub fn getF(self: PathNode) f32 {
        return self.g + self.h;
    }

    /// Sets the node's connection.
    pub fn setConnection(self: *PathNode, node: *PathNode) void {
        self.connection = node;
    }

    /// Cache all neighbouring nodes.
    pub fn cacheNeighbours(
        self: *PathNode,
        pos: Vector2,
        diagonal: bool,
        non_walkable_tiles: *std.ArrayList(Vector2),
    ) !void {
        self.neighbours.clearAndFree(self.arena_alloc);

        // cache all orthogonal neighbours first
        for (getDirections(.orthogonal)) |dir| {
            // get the neighbouring tile
            const tile = pos.add(dir);

            if (!vec2IsWalkable(tile, non_walkable_tiles)) {
                continue;
            }

            const node = PathNode.init(tile, self.arena_alloc) catch continue;
            try self.neighbours.append(self.arena_alloc, node);
        }

        // if we allow diagonal movement cache the diagonal neighbours
        if (diagonal) {
            for (getDirections(.diagonal)) |dir| {
                // get the neighbouring tile
                const tile = pos.add(dir);

                if (!vec2IsWalkable(tile, non_walkable_tiles)) {
                    continue;
                }

                const node = PathNode.init(tile, self.arena_alloc) catch continue;
                try self.neighbours.append(self.arena_alloc, node);
            }
        }
    }

    fn getDirections(movement: Movement) [4]Vector2 {
        return switch (movement) {
            .orthogonal => .{ .init(0, 1), .init(-1, 0), .init(0, -1), .init(1, 0) },
            .diagonal => .{ .init(1, 1), .init(1, -1), .init(-1, -1), .init(-1, 1) },
        };
    }

    /// Returns whether the path node is walkable and can be
    /// included in the pathfinding process.
    pub fn isWalkable(
        self: PathNode,
        non_walkable_tiles: *std.ArrayList(Vector2),
    ) bool {
        // if the list is empty it most likey means everything is walkable.
        if (non_walkable_tiles.items.len <= 0) {
            return true;
        }

        for (non_walkable_tiles.items) |non_walkable| {
            if (std.meta.eql(non_walkable, self.pos)) {
                return false;
            }
        }
        return true;
    }
};

pub fn init(alloc: std.mem.Allocator, opts: Option) !*Pathfinder {
    const self = try alloc.create(Pathfinder);

    var nwt = std.ArrayList(Vector2).empty;

    self.* = .{
        .arena = .init(alloc),
        .alloc = self.arena.allocator(),
        .work_arena = .init(alloc),
        .work_alloc = self.work_arena.allocator(),
        .path = .{ .non_walkable_tiles = &nwt },
        .opts = opts,
    };

    if (opts.use_separate_thead) {
        self.thread = try .spawn(.{ .allocator = alloc }, worker, .{self});
    }

    return self;
}

fn worker(self: *Pathfinder) void {
    while (true) {
        self.mutex.lock();
        {
            // wait for a job
            while (!self.working and !self.quit) {
                self.cond.wait(&self.mutex);
            }

            if (self.quit) {
                self.mutex.unlock();
                return;
            }

            self.working = false;
        }
        self.mutex.unlock();

        self.find(self.work_alloc) catch {
            return;
        };

        if (!self.path.found) {
            continue;
        }

        if (self.path.nodes) |nodes| {
            self.mutex.lock();
            defer self.mutex.unlock();
            self.result = self.cloneResult(nodes);
        }
    }
}

/// Attempts to find the best path from `start` to `target` with the
/// given `opts`.
pub fn findPath(
    self: *Pathfinder,
    start: @Vector(2, f32),
    target: @Vector(2, f32),
    opts: PathOption,
) !void {
    self.path.reset();
    self.path.opts = opts;
    self.path.non_walkable_tiles = &self.non_walkable_tiles;
    self.path.start_node = try .init(.initVec(start), self.work_alloc);
    self.path.target_node = try .init(.initVec(target), self.work_alloc);

    if (self.opts.use_separate_thead) {
        self.working = true;
        // send signal to the thread to start a search
        self.cond.signal();
    } else {
        try self.find(self.work_alloc);
        if (self.path.found) {
            self.result = self.cloneResult(self.path.nodes.?);
        }
    }
}

pub fn find(self: *Pathfinder, arena_alloc: std.mem.Allocator) !void {
    const is_in_bounds = self.path.start_node.pos.x <= self.grid_width and
        self.path.target_node.pos.y <= self.grid_height;

    if (!self.path.target_node.isWalkable(&self.non_walkable_tiles) or
        !is_in_bounds)
    {
        return;
    }

    switch (self.path.opts.algorithm) {
        .astar => try AStar.init(arena_alloc, &self.path),
    }
}

/// Returns the result of a pathfinding process or null if no path
/// could be found and cleans up temporary allocations used for pathfinding.
pub fn getResult(self: *Pathfinder) ?[]PathNode {
    if (self.opts.use_separate_thead) {
        if (!self.mutex.tryLock()) {
            return null;
        }
        defer self.mutex.unlock();

        // if self.working is true at this point it most likely means
        // the worker got a little overwhelmed by too many requests.
        // So we simply reset the state.
        if (self.working == true) {
            self.working = false;
        }
    }

    if (self.result != null) {
        const path = self.result;
        self.result = null;
        self.path.reset();
        _ = self.work_arena.reset(.retain_capacity);
        return path;
    }

    return null;
}

/// cloneResult copies the path result to the persistent arena
fn cloneResult(self: *Pathfinder, nodes: []PathNode) ?[]PathNode {
    const cloned = self.alloc.alloc(PathNode, nodes.len) catch {
        return null;
    };
    @memcpy(cloned, nodes);
    return cloned;
}

/// Mark a single tile as non-walkable
pub fn setNonWalkableTile(self: *Pathfinder, tile: @Vector(2, f32)) !void {
    const t: Vector2 = .initVec(tile);
    try self.non_walkable_tiles.append(self.alloc, t);
}

/// Set the tiles that should not be walkable, every other tile is
/// automatically walkable.
pub fn setNonWalkableTiles(
    self: *Pathfinder,
    tiles: []@Vector(2, f32),
) !void {
    for (tiles) |tile| {
        try self.setNonWalkableTile(tile);
    }
}

pub fn setGridSize(self: *Pathfinder, width: u32, height: u32) void {
    self.grid_width = @floatFromInt(width);
    self.grid_height = @floatFromInt(height);
}

pub fn deinit(self: *Pathfinder) void {
    self.mutex.lock();
    self.quit = true;
    self.cond.signal();
    self.mutex.unlock();

    if (self.thread) |thread| {
        thread.join();
    }

    self.arena.deinit();
    self.work_arena.deinit();
}

fn vec2IsWalkable(
    vec2: Vector2,
    non_walkable_tiles: *std.ArrayList(Vector2),
) bool {
    for (non_walkable_tiles.items) |non_walkable| {
        if (std.meta.eql(non_walkable, vec2)) {
            return false;
        }
    }
    return true;
}

/// Returns whether a PathNode ArrayList (`haystack`) contains a specific
/// node (`needle`).
pub fn nodesContain(
    haystack: std.ArrayList(*PathNode),
    needle: *PathNode,
) bool {
    for (haystack.items) |node| {
        if (node.pos.x == needle.pos.x and
            node.pos.y == needle.pos.y)
        {
            return true;
        }
    }
    return false;
}

/// Removes a PathNode from a PathNode ArrayList,
pub fn removeNode(paths: *std.ArrayList(*PathNode), node: *PathNode) void {
    var index: ?usize = null;

    // determine the index of the given PathNode
    for (paths.items, 0..) |item, i| {
        if (item.pos.x != node.pos.x and item.pos.y != node.pos.y) {
            continue;
        }
        index = i;
        break;
    }

    if (index) |idx| {
        for (idx + 1..paths.items.len) |i| {
            paths.items[i - 1] = paths.items[i];
        }
        paths.items.len -= 1;
    }
}

pub fn reverseArrayList(
    T: type,
    alloc: std.mem.Allocator,
    list: *std.ArrayList(T),
) !std.ArrayList(T) {
    var reversed = std.ArrayList(T).empty;
    var i = list.items.len;

    while (i > 0) {
        i -= 1;
        try reversed.append(alloc, list.items[i]);
    }

    return reversed;
}

/// Just to make life a little easier.
pub const Vector2 = struct {
    x: f32,
    y: f32,

    pub fn init(x: f32, y: f32) Vector2 {
        return .{ .x = x, .y = y };
    }

    pub fn initVec(vec: @Vector(2, f32)) Vector2 {
        return .{ .x = vec[0], .y = vec[1] };
    }

    pub fn add(self: Vector2, vec: Vector2) Vector2 {
        return .{ .x = self.x + vec.x, .y = self.y + vec.y };
    }

    pub fn eql(self: Vector2, vec: Vector2) bool {
        return self.x == vec.x and self.y == vec.y;
    }

    pub fn toBuiltInType(self: Vector2) @Vector(2, f32) {
        return .{ self.x, self.y };
    }
};

test "find the shortest path" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.testing.expect(gpa.deinit() == .leak) catch {};
    const alloc = gpa.allocator();

    const pf = try Pathfinder.init(alloc, .{});
    defer alloc.destroy(pf);
    defer pf.deinit();

    pf.setGridSize(40, 30);

    var non_walkable = [_]@Vector(2, f32){
        .{ 8, 15 },
        .{ 9, 15 },
        .{ 10, 15 },
        .{ 11, 15 },
    };

    try pf.setNonWalkableTiles(&non_walkable);

    try pf.findPath(.{ 10, 10 }, .{ 10, 20 }, .{
        .movement = .orthogonal,
        .heuristic = .manhattan,
    });

    const path = pf.getResult();
    try std.testing.expect(path != null);

    if (path) |p| {
        // since we're using orthogonal movement with manhattan heuristic
        // we should, in this case, get more than 10 PathNodes because we're
        // trying to avoid .{ .x = 10, .y = 15 }
        try std.testing.expect(p.len > 10);
    }
}

test "find the shortest path (threaded)" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer std.testing.expect(gpa.deinit() == .leak) catch {};
    const alloc = gpa.allocator();

    const pf = try Pathfinder.init(alloc, .{ .use_separate_thead = true });
    defer alloc.destroy(pf);
    defer pf.deinit();

    pf.setGridSize(40, 30);

    try pf.findPath(.{ 10, 10 }, .{ 10, 20 }, .{});

    var path: ?[]PathNode = null;
    const deadline = std.time.milliTimestamp() + 200;

    while (path == null and std.time.milliTimestamp() < deadline) {
        path = pf.getResult();
        std.Thread.sleep(1 * std.time.ns_per_ms);
    }

    try std.testing.expect(path != null);

    if (path) |p| {
        try std.testing.expect(p.len > 0);
    }
}
