#pragma once
/// @file BVHTree.h
/// @brief Dynamic AABB bounding volume hierarchy with SAH-guided insertion.

#include "physics/AABB.h"
#include <vector>
#include <unordered_map>
#include <utility>

namespace physics {

// ── Internal node ─────────────────────────────────────────────────────────

/// A single node in the BVH tree (both internal nodes and leaves share this struct).
struct BVHNode {
    AABB aabb;              ///< Tight AABB for leaves; union of children for internals
    int  parent    = -1;   ///< Parent index; -1 for root
    int  left      = -1;   ///< Left child index; -1 means this is a leaf
    int  right     = -1;   ///< Right child index; -1 means this is a leaf
    int  bodyIndex = -1;   ///< Body index for leaves; -1 for internal nodes
    int  nextFree  = -1;   ///< Free-list linkage (unused when allocated)

    bool isLeaf() const { return left == -1; }
};

// ── BVH Tree ──────────────────────────────────────────────────────────────

/// Dynamic AABB tree using SAH branch-and-bound insertion.
/// Stores body indices (not pointers) and is decoupled from RigidBody internals.
class BVHTree {
public:
    BVHTree() = default;

    // ── Mutation ──────────────────────────────────────────────────────────

    /// Insert a leaf for the given body with the given world-space AABB.
    /// The stored AABB is fattened by fattenMargin_ for update stability.
    void insert(int bodyIndex, const AABB& aabb);

    /// Remove the leaf for the given body. No-op if not found.
    void remove(int bodyIndex);

    /// Update the AABB for the given body.
    /// If the new AABB is still contained in the stored fat AABB, no reinsert.
    void update(int bodyIndex, const AABB& newAABB);

    /// Remove all nodes and reset the tree.
    void clear();

    // ── Queries ───────────────────────────────────────────────────────────

    /// Return all overlapping leaf-pair indices (i < j).
    /// Static-static pairs are NOT excluded here; filter at call site.
    std::vector<std::pair<int, int>> queryAllPairs() const;

    /// Return bodyIndices of all leaves whose AABB overlaps the given query AABB.
    std::vector<int> queryAABB(const AABB& query) const;

    // ── Metrics / debug ───────────────────────────────────────────────────

    int   nodeCount()       const { return static_cast<int>(nodes_.size()) - freeCount_; }
    int   leafCount()       const { return leafCount_; }
    float totalSurfaceArea()const;                 ///< Sum of all node SA (quality metric)
    void  validate()        const;                 ///< Assert tree invariants (debug)

    // ── Configuration ─────────────────────────────────────────────────────

    float fattenMargin() const              { return fattenMargin_; }
    void  setFattenMargin(float m)          { fattenMargin_ = m; }

private:
    std::vector<BVHNode>         nodes_;          ///< Node pool (indexed by int)
    int                          root_       = -1;
    int                          freeList_   = -1; ///< Head of recycled-node list
    int                          freeCount_  = 0;  ///< Number of nodes on free list
    int                          leafCount_  = 0;
    float                        fattenMargin_ = 0.1f;
    std::unordered_map<int, int> bodyToNode_;     ///< bodyIndex → nodeIndex

    // ── Internal helpers ─────────────────────────────────────────────────

    int  allocNode();
    void freeNode(int index);

    void insertLeaf(int leafIndex);
    void removeLeaf(int leafIndex);

    /// SAH branch-and-bound: find the best sibling node for a new leaf.
    int  findBestSibling(const AABB& leafAABB) const;

    /// Walk from nodeIndex up to root, recomputing AABBs.
    void refitAncestors(int nodeIndex);

    /// Collect overlapping leaf pairs by descending a pair of subtrees.
    void traversePairs(int a, int b,
                       std::vector<std::pair<int, int>>& out) const;
};

} // namespace physics
