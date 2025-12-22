///! The A* pathfinding algorithm.
const AStar = @This();
const std = @import("std");

const pf = @import("pathfinder.zig");
const Path = pf.Path;
const PathNode = pf.PathNode;

alloc: std.mem.Allocator,

/// `PathNodes` that need to be searched.
to_search: std.ArrayList(*PathNode) = .empty,

/// `PathNodes` that have already been searched.
processed: std.ArrayList(*PathNode) = .empty,

/// Whether diagonal movement should be allowed.
diagonal: bool,

/// The PathNode of the starting position.
start_node: *PathNode,

/// The PathNode of the target position.
target_node: *PathNode,

current: *PathNode = undefined,
path: *Path,

/// Heuristics for the A* algorithm.
pub const Heuristic = enum {
    /// Best grid-based movement restricted to horizontal and vertical
    /// directions.
    manhattan,

    /// Best grids where diagonal movement is allowed.
    euclidian,

    /// Best for 8-directional movement. (Horizontally and vertically)
    chebyshev,

    /// Get the distance between `start` and `target` using the given heuristic.
    pub fn getDist(self: Heuristic, start: pf.Vector2, target: pf.Vector2) f32 {
        const dist_x = start.x - target.x;
        const dist_y = start.y - target.y;

        return switch (self) {
            .manhattan => {
                return @abs(dist_x) + @abs(dist_y);
            },
            .euclidian => {
                return @sqrt(dist_x * dist_x + dist_y * dist_y);
            },
            .chebyshev => {
                return @max(@abs(dist_x), @abs(dist_y));
            },
        };
    }
};

pub fn init(alloc: std.mem.Allocator, path: *Path) anyerror!void {
    var self: AStar = .{
        .alloc = alloc,
        .diagonal = path.opts.movement == .diagonal,
        .path = path,
        .start_node = path.start_node,
        .target_node = path.target_node,
    };

    try self.find();
}

fn find(self: *AStar) !void {
    // get the neighbouring nodes for start and target position
    try self.start_node.cacheNeighbours(
        self.start_node.pos,
        self.diagonal,
        self.path.non_walkable_tiles,
    );
    try self.target_node.cacheNeighbours(
        self.target_node.pos,
        self.diagonal,
        self.path.non_walkable_tiles,
    );

    // Append the start node to the search list
    // so that we can start the search.
    try self.to_search.append(self.alloc, self.start_node);

    while (self.to_search.items.len > 0) {
        self.current = self.to_search.items[0];

        // iterate through the search list and get the most appropriate
        // neighbouring node
        for (self.to_search.items) |t| {
            if (t.getF() < self.current.getF() or
                t.getF() == self.current.getF() and t.h < self.current.h)
            {
                self.current = t;
            }
        }

        self.processed.append(self.alloc, self.current) catch continue;
        pf.removeNode(&self.to_search, self.current);

        // if we reached the target position build the final path.
        if (self.current.pos.eql(self.path.target_node.pos)) {
            try self.buildPath();
            return;
        }

        // Iterate through all the neighbouring nodes and determine the cost
        for (self.current.neighbours.items) |neighbour| {
            if (!neighbour.isWalkable(self.path.non_walkable_tiles) and
                pf.nodesContain(self.processed, neighbour))
            {
                continue;
            }

            try self.findConnection(neighbour);
        }
    }
}

/// Builds the final path list and updates the path object.
fn buildPath(self: *AStar) !void {
    var current_path_tile = self.current;
    var path_nodes: std.ArrayList(PathNode) = .empty;

    // Walk backwards from the current node (which should be the
    // target node) to the starting node by following the
    // connected nodes.
    while (!current_path_tile.pos.eql(self.path.start_node.pos)) {
        if (self.path.opts.edge_centered) {
            current_path_tile.pos.x += 0.5;
            current_path_tile.pos.y += 0.5;
        }

        try path_nodes.append(self.alloc, current_path_tile.*);

        if (current_path_tile.connection) |connection| {
            current_path_tile = connection;
        }
    }

    // Reverse the list because we generally want to have the
    // path ordered from the start position.
    var p = try pf.reverseArrayList(PathNode, self.alloc, &path_nodes);

    self.path.update(try p.toOwnedSlice(self.alloc));
}

/// Finds the best connection got a node
fn findConnection(self: *AStar, node: *PathNode) !void {
    const current = self.current;
    const in_search = pf.nodesContain(self.to_search, node);

    var dist = self.path.opts.heuristic.getDist(
        current.pos,
        node.pos,
    );
    const cost_to_heighbour = current.g + @round(dist);

    // set conncetions of it's the node with the lowest cost
    if (!in_search or cost_to_heighbour < node.g) {
        node.setG(cost_to_heighbour);
        node.setConnection(current);
    }

    if (!in_search) {
        dist = self.path.opts.heuristic.getDist(
            node.pos,
            self.target_node.pos,
        );

        node.setH(@round(dist));
        node.cacheNeighbours(
            node.pos,
            self.diagonal,
            self.path.non_walkable_tiles,
        ) catch |e| {
            std.log.err("Failed to cache neighbouring nodes. {}", .{e});
            return;
        };

        try self.to_search.append(self.alloc, node);
    }
}
