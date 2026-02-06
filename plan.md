# Bulk Primitive Refactor Plan (Staged, Fully Operational at Each Milestone)

## Summary
We will move ScenePart geometry from per-primitive heap allocations to bulk, STL-backed arrays stored directly in `ScenePart`, while preserving correctness and performance at each milestone. We will introduce bulk storage types, add non-owning adapters to keep existing `Primitive*` APIs working during migration, then progressively update subsystems (KD-tree, raycasting, dynamics, visualizers, Python bindings) to use bulk data directly, and finally remove the adapter layer and legacy APIs.

## Milestone 1 — Bulk Geometry Model + Converters (No Runtime Behavior Change)
**Goal:** Add bulk storage structures and conversion utilities without changing how the system operates.

1. Add new bulk storage types (new header, e.g., `src/scene/ScenePartGeometry.h`):
   - `TriangleBulk`:
     - `std::vector<Vertex> vertices` (size = 3 * n)
     - `std::vector<glm::dvec3> face_normal` (size = n)
     - `std::vector<glm::dvec3> e1, e2, v0` (size = n)
     - `std::vector<double> eps` (size = n, default = `1e-7`)
     - `std::vector<AABB> aabbs` (size = n) or `min/max` arrays
     - `std::vector<std::shared_ptr<Material>> materials` (size = n)
   - `VoxelBulk`:
     - `std::vector<Vertex> centers` (size = n)
     - `std::vector<double> half_size` (size = n)
     - `std::vector<int> num_points` (size = n)
     - `std::vector<double> r, g, b` (size = n)
     - `std::vector<Color4f> color` (size = n)
     - `std::vector<AABB> aabbs` (size = n)
     - `std::vector<std::shared_ptr<Material>> materials` (size = n)
   - `DetailedVoxelBulk` (optional wrapper):
     - `std::vector<std::vector<int>> int_values`
     - `std::vector<std::vector<double>> double_values`
     - `std::vector<double> max_pad`
   - `using PrimitiveIndex = std::size_t;`
2. Add bulk fields to `ScenePart`:
   - `TriangleBulk triangles;`
   - `VoxelBulk voxels;`
   - `DetailedVoxelBulk detailed_voxels;` (only used when needed)
   - `PrimitiveType primitiveType` remains the discriminator.
3. Add conversion utilities in `ScenePart`:
   - `buildBulkFromPrimitives()` (reads `mPrimitives` and fills bulk)
   - `buildPrimitivesFromBulk()` (optional, for debug/bridging only)
4. No behavioral changes yet:
   - Keep `mPrimitives` as the authoritative data source for now.
   - Add unit-style checks (in C++ tests) that bulk mirrors primitives correctly.

**Operational checkpoint:** All existing code works unchanged; bulk data is generated and validated but not used for execution.

## Milestone 2 — Bulk Storage Becomes Source of Truth + Primitive Adapters
**Goal:** Remove per-primitive heap allocations while keeping existing APIs operational.

1. Switch geometry loaders to build bulk directly:
   - Update `WavefrontObjFileLoader`, `DetailedVoxelLoader`, `XYZPointCloudFileLoader`, `GeoTiffFileLoader`, and any geometry filters that create primitives to fill `ScenePart` bulk arrays instead.
2. Introduce non-owning adapter views:
   - `TriangleView`, `VoxelView`, `DetailedVoxelView` classes derive from `Primitive` and store:
     - `ScenePart* owner`
     - `PrimitiveIndex index`
   - Implement `Primitive` virtuals by reading/writing bulk arrays.
   - Store adapters by value in `ScenePart`:
     - `std::vector<TriangleView> triangleViews;`
     - `std::vector<VoxelView> voxelViews;`
     - `std::vector<DetailedVoxelView> detailedVoxelViews;`
   - Build `mPrimitives` as a non-owning pointer list to these views.
   - Guarantee pointer stability by sizing view vectors once per rebuild.
3. Update `ScenePart` operations to operate on bulk:
   - `getAllVertices`, `smoothVertexNormals`, `computeCentroid`, `computeTransformations`, `splitSubparts`, `release`.
4. Update SwapOnRepeatHandler and cloning paths:
   - Clone/copy bulk arrays instead of heap primitives.
   - Rebuild views after swaps.
5. Serialization:
   - Add versioning to `ScenePart` and `Scene` serialization.
   - When loading legacy serialized scenes, convert `mPrimitives` to bulk and then build views.

**Operational checkpoint:** No per-primitive heap allocations for triangles/voxels. Old APIs (`Primitive*`, Python wrappers, KD-tree) still function via adapters.

## Milestone 3 — Migrate Core Subsystems to Bulk APIs
**Goal:** Remove `Primitive*` from performance-critical subsystems.

1. Define index-based primitive references:
   - `struct PrimitiveRef { ScenePart* part; PrimitiveType type; PrimitiveIndex index; };`
2. KD-tree and raycasting:
   - Introduce `PrimitiveAccessor` interface with:
     - `getAABB(ref)`, `getCentroid(ref)`, `rayIntersection(ref, ...)`, `rayIntersectionDistance(ref, ...)`, `getGroundZOffset(ref)`
   - Update KD-tree factories, nodes, and raycasters to store `std::vector<PrimitiveRef>` instead of `std::vector<Primitive*>`.
3. Scene and ScenePart:
   - Replace `Scene::primitives` with `std::vector<PrimitiveRef>` or compute on demand from `ScenePart`s.
   - Update `Scene::finalizeLoading`, `getAllVertices`, `registerParts`, and AABB computations to use bulk data.
4. Dynamic objects:
   - Update `DynObject` and `DynMovingObject` matrix routines to read/write bulk arrays directly.
   - Use `arma::mat` only as a computation object (build from bulk, operate, write back).
5. Visualizers and adapters:
   - Update `VHStaticObjectAdapter`, `VHSimpleCanvas`, and scene adapters to iterate bulk arrays.
6. Python bindings:
   - Add bulk accessors (e.g., `ScenePart.triangle_vertices`, `ScenePart.voxel_centers`).
   - Keep `Primitive`-based API for now (backed by adapters) but mark it as transitional.

**Operational checkpoint:** Performance-critical systems no longer rely on `Primitive*`. Adapters still exist for compatibility.

## Milestone 4 — Remove Legacy Primitive Adapters and Finalize APIs
**Goal:** Fully remove `Primitive*` usage from Scene/ScenePart and clean up interfaces.

1. Remove `ScenePart::mPrimitives` and adapter view vectors.
2. Deprecate or remove `Primitive` exposure in C++ and Python:
   - Replace with bulk APIs and index-based helpers.
3. Finalize serialization on bulk-only schema:
   - Optionally keep read-compat layer for legacy scenes if needed.
4. Clean up build/tests and remove now-unused code paths.

**Operational checkpoint:** Scene geometry is fully stored in bulk arrays only. No `Primitive*` dependency in runtime systems.

## Important API / Type Changes (Planned)
- New bulk storage types: `TriangleBulk`, `VoxelBulk`, optional `DetailedVoxelBulk`.
- New index type alias: `PrimitiveIndex = std::size_t`.
- New adapter classes: `TriangleView`, `VoxelView`, `DetailedVoxelView` (temporary).
- New index-based reference: `PrimitiveRef`.
- New ScenePart bulk accessors:
  - `triangleCount()`, `triangleVertices(i)`, `triangleMaterial(i)`, etc.
  - `voxelCount()`, `voxelCenter(i)`, `voxelHalfSize(i)`, etc.
- Updated serialization versioning for `ScenePart` and `Scene`.

## Tests and Scenarios
1. Existing C++ tests in `src/test` (especially `ScenePartSplitTest`, `RigidMotionTest`, `SerializationTest`, `RayIntersectionTest`).
2. Python tests in `tests/python`.
3. Manual scenario checks:
   - Load OBJ -> finalize -> KD-tree build -> raycast intersections.
   - Dynamic moving object updates (position/normal matrix updates).
   - Swap-on-repeat behavior with baseline geometry.
   - Serialization round-trip (old -> new -> old data invariants).

## Assumptions and Defaults
- ScenePart contains a single `PrimitiveType` at a time (as currently implied).
- Triangles use per-triangle vertices (3*n) with no vertex sharing.
- Primary storage uses STL containers; `arma::mat` is used only for computations.
- Backward compatibility for serialized scenes is maintained (we can drop if desired).
- DetailedVoxel per-voxel vectors remain as vectors initially; packed storage can be a later optimization.
