/// @file BVHTree.cpp
/// @brief Dynamic AABB BVH with SAH branch-and-bound insertion.

#include "physics/BVHTree.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <queue>
#include <stack>
#include <limits>

namespace physics {

// ── Free list allocator ───────────────────────────────────────────────────

int BVHTree::allocNode() {
    if (freeList_ != -1) {
        const int idx = freeList_;
        freeList_ = nodes_[idx].nextFree;
        --freeCount_;
        nodes_[idx] = BVHNode{};   // reset
        return idx;
    }
    nodes_.push_back(BVHNode{});
    return static_cast<int>(nodes_.size()) - 1;
}

void BVHTree::freeNode(int index) {
    assert(index >= 0 && index < static_cast<int>(nodes_.size()));
    nodes_[index].nextFree = freeList_;
    freeList_ = index;
    ++freeCount_;
}

// ── Public insert / remove / update ──────────────────────────────────────

void BVHTree::insert(int bodyIndex, const AABB& aabb) {
    // Fatten the AABB for stability
    const Vec3 margin(fattenMargin_, fattenMargin_, fattenMargin_);
    const AABB fat{ aabb.min - margin, aabb.max + margin };

    const int leaf = allocNode();
    nodes_[leaf].aabb      = fat;
    nodes_[leaf].bodyIndex = bodyIndex;
    nodes_[leaf].parent    = -1;
    nodes_[leaf].left      = -1;
    nodes_[leaf].right     = -1;

    bodyToNode_[bodyIndex] = leaf;
    ++leafCount_;

    insertLeaf(leaf);
}

void BVHTree::remove(int bodyIndex) {
    auto it = bodyToNode_.find(bodyIndex);
    if (it == bodyToNode_.end()) return;
    const int leaf = it->second;
    bodyToNode_.erase(it);
    --leafCount_;
    removeLeaf(leaf);
    freeNode(leaf);
}

void BVHTree::update(int bodyIndex, const AABB& newAABB) {
    auto it = bodyToNode_.find(bodyIndex);
    if (it == bodyToNode_.end()) return;
    const int leaf = it->second;

    // Fast-path: if new AABB fits inside the stored fat AABB, no reinsert
    const AABB& stored = nodes_[leaf].aabb;
    if (stored.min.x <= newAABB.min.x && stored.min.y <= newAABB.min.y &&
        stored.min.z <= newAABB.min.z && stored.max.x >= newAABB.max.x &&
        stored.max.y >= newAABB.max.y && stored.max.z >= newAABB.max.z) {
        return; // fat AABB still covers the tight new AABB
    }

    // Remove old leaf, create new one with fattened AABB
    removeLeaf(leaf);
    freeNode(leaf);

    const Vec3 margin(fattenMargin_, fattenMargin_, fattenMargin_);
    const AABB fat{ newAABB.min - margin, newAABB.max + margin };

    const int newLeaf = allocNode();
    nodes_[newLeaf].aabb      = fat;
    nodes_[newLeaf].bodyIndex = bodyIndex;
    nodes_[newLeaf].parent    = -1;
    nodes_[newLeaf].left      = -1;
    nodes_[newLeaf].right     = -1;

    bodyToNode_[bodyIndex] = newLeaf;
    insertLeaf(newLeaf);
}

void BVHTree::clear() {
    nodes_.clear();
    bodyToNode_.clear();
    root_      = -1;
    freeList_  = -1;
    freeCount_ = 0;
    leafCount_ = 0;
}

// ── SAH sibling search (branch-and-bound) ────────────────────────────────

int BVHTree::findBestSibling(const AABB& leafAABB) const {
    assert(root_ != -1);
    const float leafSA = leafAABB.surfaceArea();

    // Min-heap: (lowerBoundCost, nodeIndex)
    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    float bestCost = std::numeric_limits<float>::max();
    int   bestNode = root_;

    pq.push({ leafAABB.merged(nodes_[root_].aabb).surfaceArea(), root_ });

    while (!pq.empty()) {
        auto [lbCost, idx] = pq.top(); pq.pop();

        // Prune: lower bound already exceeds best
        if (lbCost >= bestCost) break;

        const BVHNode& node = nodes_[idx];
        const float directCost = leafAABB.merged(node.aabb).surfaceArea();

        // Inherited cost: how much the ancestors' AABBs would grow
        float inheritedCost = 0.0f;
        {
            int ancestor = node.parent;
            while (ancestor != -1) {
                const AABB enlarged = leafAABB.merged(nodes_[ancestor].aabb);
                inheritedCost += enlarged.surfaceArea() - nodes_[ancestor].aabb.surfaceArea();
                ancestor = nodes_[ancestor].parent;
            }
        }

        const float totalCost = directCost + inheritedCost;
        if (totalCost < bestCost) {
            bestCost = totalCost;
            bestNode = idx;
        }

        // Push children if their lower bound could beat bestCost
        if (!node.isLeaf()) {
            // Lower bound for a child: insertedLeaf SA + currentInheritedCost + delta SA of this node
            const float childInherited = inheritedCost +
                (leafAABB.merged(node.aabb).surfaceArea() - node.aabb.surfaceArea());
            const float childLowerBound = leafSA + childInherited;
            if (childLowerBound < bestCost) {
                pq.push({ childLowerBound, node.left  });
                pq.push({ childLowerBound, node.right });
            }
        }
    }

    return bestNode;
}

// ── Internal tree operations ──────────────────────────────────────────────

void BVHTree::insertLeaf(int leafIndex) {
    if (root_ == -1) {
        root_ = leafIndex;
        nodes_[leafIndex].parent = -1;
        return;
    }

    // 1. Find best sibling
    const int sibling = findBestSibling(nodes_[leafIndex].aabb);

    // 2. Create new internal parent
    const int oldParent = nodes_[sibling].parent;
    const int newParent = allocNode();
    nodes_[newParent].aabb   = nodes_[leafIndex].aabb.merged(nodes_[sibling].aabb);
    nodes_[newParent].parent = oldParent;
    nodes_[newParent].left   = sibling;
    nodes_[newParent].right  = leafIndex;
    nodes_[newParent].bodyIndex = -1;

    // 3. Link new parent into the tree
    if (oldParent == -1) {
        root_ = newParent;
    } else {
        if (nodes_[oldParent].left == sibling)
            nodes_[oldParent].left  = newParent;
        else
            nodes_[oldParent].right = newParent;
    }

    // 4. Update child parents
    nodes_[sibling].parent   = newParent;
    nodes_[leafIndex].parent = newParent;

    // 5. Refit ancestors
    refitAncestors(newParent);
}

void BVHTree::removeLeaf(int leafIndex) {
    if (leafIndex == root_) {
        root_ = -1;
        return;
    }

    const int parent      = nodes_[leafIndex].parent;
    const int grandparent = nodes_[parent].parent;
    const int sibling     = (nodes_[parent].left == leafIndex)
                            ? nodes_[parent].right
                            : nodes_[parent].left;

    if (grandparent == -1) {
        // Parent was root → sibling becomes root
        root_ = sibling;
        nodes_[sibling].parent = -1;
    } else {
        // Graft sibling into grandparent
        if (nodes_[grandparent].left == parent)
            nodes_[grandparent].left  = sibling;
        else
            nodes_[grandparent].right = sibling;
        nodes_[sibling].parent = grandparent;
        refitAncestors(grandparent);
    }

    freeNode(parent);
}

void BVHTree::refitAncestors(int nodeIndex) {
    int idx = nodeIndex;
    while (idx != -1) {
        BVHNode& n = nodes_[idx];
        if (!n.isLeaf()) {
            n.aabb = nodes_[n.left].aabb.merged(nodes_[n.right].aabb);
        }
        idx = n.parent;
    }
}

// ── Queries ───────────────────────────────────────────────────────────────

void BVHTree::traversePairs(int a, int b,
                             std::vector<std::pair<int, int>>& out) const {
    if (a == -1 || b == -1) return;

    // Self-traversal: enumerate all pairs within a single subtree
    if (a == b) {
        const BVHNode& n = nodes_[a];
        if (n.isLeaf()) return; // single leaf → no pairs
        // Recurse into: left×left, left×right, right×right
        traversePairs(n.left,  n.left,  out);
        traversePairs(n.left,  n.right, out);
        traversePairs(n.right, n.right, out);
        return;
    }

    // Cross-traversal: find all overlapping pairs between subtree A and subtree B
    const BVHNode& na = nodes_[a];
    const BVHNode& nb = nodes_[b];

    if (!na.aabb.overlaps(nb.aabb)) return;

    if (na.isLeaf() && nb.isLeaf()) {
        // emit ordered pair
        const int lo = std::min(na.bodyIndex, nb.bodyIndex);
        const int hi = std::max(na.bodyIndex, nb.bodyIndex);
        out.emplace_back(lo, hi);
        return;
    }

    // Descend into the larger node so recursion terminates naturally
    if (na.isLeaf()) {
        traversePairs(a, nb.left,  out);
        traversePairs(a, nb.right, out);
    } else if (nb.isLeaf()) {
        traversePairs(na.left,  b, out);
        traversePairs(na.right, b, out);
    } else {
        if (na.aabb.surfaceArea() >= nb.aabb.surfaceArea()) {
            traversePairs(na.left,  b, out);
            traversePairs(na.right, b, out);
        } else {
            traversePairs(a, nb.left,  out);
            traversePairs(a, nb.right, out);
        }
    }
}

std::vector<std::pair<int, int>> BVHTree::queryAllPairs() const {
    std::vector<std::pair<int, int>> result;
    if (root_ == -1 || leafCount_ < 2) return result;

    // Self-collision traversal — correctly enumerates all leaf pairs
    traversePairs(root_, root_, result);

    // Deduplicate (cross-traversal with equal subtrees won't produce dupes, but be safe)
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

std::vector<int> BVHTree::queryAABB(const AABB& query) const {
    std::vector<int> result;
    if (root_ == -1) return result;

    std::stack<int> stack;
    stack.push(root_);

    while (!stack.empty()) {
        const int idx = stack.top(); stack.pop();
        const BVHNode& node = nodes_[idx];

        if (!node.aabb.overlaps(query)) continue;

        if (node.isLeaf()) {
            result.push_back(node.bodyIndex);
        } else {
            stack.push(node.left);
            stack.push(node.right);
        }
    }
    return result;
}

// ── Metrics ───────────────────────────────────────────────────────────────

float BVHTree::totalSurfaceArea() const {
    float total = 0.0f;
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        // Skip free-list nodes — they have nextFree set but we can't easily
        // distinguish them without another flag; just sum all allocated AABBs.
        // Since free nodes are zeroed, their SA = 0,which is safe to sum.
        (void)i;
    }
    // Walk tree for accurate count
    if (root_ == -1) return 0.0f;
    std::stack<int> stack;
    stack.push(root_);
    while (!stack.empty()) {
        const int idx = stack.top(); stack.pop();
        total += nodes_[idx].aabb.surfaceArea();
        if (!nodes_[idx].isLeaf()) {
            stack.push(nodes_[idx].left);
            stack.push(nodes_[idx].right);
        }
    }
    return total;
}

// ── Validation ────────────────────────────────────────────────────────────

void BVHTree::validate() const {
#ifndef NDEBUG
    if (root_ == -1) {
        assert(leafCount_ == 0);
        return;
    }

    // Iterative DFS — check invariants at each node
    std::stack<std::pair<int, int>> stack; // (nodeIndex, depth)
    stack.push({root_, 0});

    const int maxDepth = static_cast<int>(nodes_.size()) + 1;

    while (!stack.empty()) {
        auto [idx, depth] = stack.top(); stack.pop();
        assert(depth < maxDepth && "BVH cycle detected");

        const BVHNode& n = nodes_[idx];

        if (n.isLeaf()) {
            assert(n.bodyIndex >= 0 && "Leaf must have valid bodyIndex");
            assert(n.left  == -1 && n.right == -1);
        } else {
            assert(n.bodyIndex == -1 && "Internal node must not have bodyIndex");
            assert(n.left  != -1 && n.right != -1);

            // Verify AABB is union of children
            const AABB expected = nodes_[n.left].aabb.merged(nodes_[n.right].aabb);
            assert(std::fabs(n.aabb.min.x - expected.min.x) < 1e-3f &&
                   std::fabs(n.aabb.min.y - expected.min.y) < 1e-3f &&
                   std::fabs(n.aabb.min.z - expected.min.z) < 1e-3f &&
                   std::fabs(n.aabb.max.x - expected.max.x) < 1e-3f &&
                   std::fabs(n.aabb.max.y - expected.max.y) < 1e-3f &&
                   std::fabs(n.aabb.max.z - expected.max.z) < 1e-3f &&
                   "Internal AABB must be union of children");

            // Verify parent-child links
            assert(nodes_[n.left].parent  == idx && "Left child parent mismatch");
            assert(nodes_[n.right].parent == idx && "Right child parent mismatch");

            stack.push({n.left,  depth + 1});
            stack.push({n.right, depth + 1});
        }

        // Non-root: parent must reference this node
        if (idx != root_) {
            const int p = n.parent;
            assert(p != -1);
            assert(nodes_[p].left == idx || nodes_[p].right == idx);
        }
    }
#endif
}

} // namespace physics
