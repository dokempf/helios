# Helios++ Primitive-to-Bulk ScenePart Refactor Plan

## Goal
Refactor geometry storage from per-primitive heap allocations (`std::vector<Primitive*>`) to bulk storage owned by polymorphic `ScenePart` implementations (e.g. `TriangleScenePart`, `VoxelScenePart`), while preserving behavior for loading, ray intersection, KD-tree construction, dynamic updates, and swap-on-repeat.

## Architectural Decision (Accepted)
- `AABB` will be refactored to a standalone bounding-volume utility class and will no longer inherit from `Primitive`.
- Consequence: scene geometry primitives are limited to true scene elements (`Triangle`, `Voxel`, `DetailedVoxel`) and their bulk `ScenePart` representations.

## Current State Summary (from code inspection)
- Core ownership today:
  - `ScenePart::mPrimitives` (`src/assetloading/ScenePart.h`) owns raw `Primitive*`.
  - `Scene::primitives` (`src/scene/Scene.h`) stores global raw `Primitive*`.
  - Lifetime is manual (`ScenePart::release`, `Scene::~Scene`, swap paths).
- Primitive virtual API is used everywhere:
  - KD-tree build and traversal (`src/adt/kdtree/*`, `src/alg/raycast/*`) call `getCentroid`, `getAABB`, `getRayIntersectionDistance`, etc.
  - Scene finalization and transform paths mutate vertices via `Primitive::getVertices`.
  - Dynamic objects update vertices/normals via matrix conversion over primitives (`src/scene/dynamic/DynObject.cpp`).
- Loaders create primitives one-by-one (`new Triangle`, `new Voxel`, `new DetailedVoxel`) in:
  - OBJ, TIFF, XYZ, VOX loaders (`src/assetloading/geometryfilter/*`, `src/assetloading/VoxelFileParser.cpp`).
- Swap-on-repeat currently clones/deletes full primitive objects (`src/assetloading/SwapOnRepeatHandler.cpp`).
- Python bindings expose `Primitive`, `ScenePart.primitives`, direct primitive indexing (`python/helios/helios_python.cpp`).
- `AABB` is currently modeled as `Primitive` mostly for polymorphic convenience in serialization/tests/bindings.

## Proposed Target Architecture

### 1. ScenePart becomes polymorphic geometry owner
- Keep `ScenePart` as base class for common metadata:
  - ID, CRS, transform config, force-on-ground, swap metadata, object type.
- Add virtual geometry interface on `ScenePart` for per-element operations (index-based):
  - `size_t primitiveCount() const`
  - `PrimitiveType primitiveType() const`
  - `std::shared_ptr<AABB> computeBound() const`
  - `glm::dvec3 primitiveCentroid(size_t i) const`
  - `double primitiveRayIntersectionDistance(size_t i, const glm::dvec3&, const glm::dvec3&) const`
  - `std::vector<double> primitiveRayIntersection(size_t i, ...) const`
  - `double primitiveIncidenceAngle(size_t i, ...) const`
  - vertex/normal accessors and mutators required by dynamic code and transforms.

### 2. Concrete bulk SceneParts
- `TriangleScenePart`:
  - Bulk arrays for vertices, normals, precomputed triangle terms (`e1/e2/v0`), per-triangle AABB.
  - Constant/common fields as plain members (e.g., epsilon if shared).
  - Per-triangle material mapping via index vector.
- `VoxelScenePart`:
  - Bulk centers, half-size, bbox per voxel, RGB aggregates, optional normals.
- `DetailedVoxelScenePart`:
  - Extends voxel bulk data with integer/double matrices and identifier mapping.
- No `AABBScenePart`. `AABB` remains utility-only for bounds/intersection helpers.

### 3. Primitive compatibility strategy (staged migration)
To reduce risk while avoiding throwaway abstractions:
- Use direct target abstractions early (index-based geometry access on `ScenePart` and stable handle types that remain in final design).
- Avoid introducing migration-only layers that are planned to be deleted shortly after.
- Keep checkpoints functional by vertical slices (one primitive family end-to-end) instead of horizontal temporary adapter layers.

Recommended checkpoint style:
1. Triangle bulk path complete end-to-end (loader -> scene -> KD -> raycast -> tests).
2. Voxel bulk path complete end-to-end.
3. Detailed voxel path complete end-to-end.
4. Dynamic/swap/serialization/python convergence.

## Detailed Implementation Phases

### Phase 0: Safety rails and baseline
1. Add characterization tests before refactor changes:
   - Scene loading counts by primitive type.
   - Intersection parity for representative scenes (triangle-only, voxel-only, detailed voxel).
   - Dynamic object motion updates on small fixtures.
   - Swap-on-repeat replay geometry lifecycle invariants.
2. Add microbenchmark harness (optional but recommended) for:
   - scene load time,
   - peak memory,
   - KD build time,
   - raycast throughput.

### Phase 1: Introduce polymorphic ScenePart geometry interface
1. Extend `ScenePart` base (`src/assetloading/ScenePart.h/.cpp`) with new virtual index-based geometry API.
2. Keep existing `mPrimitives` temporarily only as migration shim.
3. Implement default `ScenePart` behavior delegating to existing `mPrimitives` so current tests continue passing.

### Phase 1.5: Decouple AABB from Primitive (new mandatory phase)
1. Refactor `AABB` to stop inheriting `Primitive`:
   - Remove `Primitive` base class from `src/scene/primitives/AABB.h`.
   - Remove `clone/_clone` overrides tied to `Primitive`.
   - Keep only pure bounding-volume API (`getMin/getMax/getRayIntersection/...`).
2. Replace all APIs that relied on `AABB` as polymorphic `Primitive`:
   - Keep `Primitive::getAABB()` returning `AABB*` (or `const AABB*`) but ensure `AABB` itself is not a `Primitive`.
3. Remove `AABB` from primitive type registration where registered as derived primitive:
   - `src/scene/Scene.h` primitive registrations.
   - `src/adt/kdtree/LightKDTreeNode.h` primitive registrations.
4. Remove any code path that inserts `AABB` into primitive containers (`Scene::primitives`, KD leaf primitive vectors).
5. Ensure no runtime logic treats AABB as hittable scene geometry primitive.

### Phase 2: Add bulk ScenePart implementations
1. Add new files:
   - `src/scene/sceneparts/TriangleScenePart.h/.cpp`
   - `src/scene/sceneparts/VoxelScenePart.h/.cpp`
   - `src/scene/sceneparts/DetailedVoxelScenePart.h/.cpp`
2. Implement geometry API using bulk arrays only (no per-primitive heap allocations).
3. Implement transformation/update hooks on bulk data.
4. Implement material indirection strategy:
   - `std::vector<std::shared_ptr<Material>> materialTable`
   - `arma::uvec materialIndex` or `std::vector<uint32_t>` per primitive.
   - Keep `std::shared_ptr<Material>` semantics (no value-copy material storage).

### Phase 3: Migrate loaders to build bulk SceneParts directly
1. OBJ/TIFF loaders produce `TriangleScenePart`.
2. XYZ loader produces `VoxelScenePart`.
3. Detailed voxel loader/VOX parser produces `DetailedVoxelScenePart`.
4. Remove intermediate `WavefrontObj::primitives` ownership model or convert it to bulk-friendly temporary representation.

### Phase 4: Replace `Scene::primitives` and KD input type
1. Introduce `Scene::primitiveRefs` (or equivalent) derived from all parts.
2. Update KD-tree build APIs (`src/adt/kdtree/*`, `src/adt/grove/KDGroveFactory.cpp`) to accept refs instead of `Primitive*`.
3. Update node storage (`LightKDTreeNode::primitives`) to store ref vectors.
4. Update comparators and SAH/split logic to query centroid/AABB through owner+index.
5. Keep the chosen handle type as a permanent part of the architecture (do not introduce a second replacement handle later).
6. Remove `Scene::primitives` entirely (not retained as compatibility view).

### Phase 5: Raycasting and intersection migration
1. Update `KDTreeRaycaster`, `GroveKDTreeRaycaster`, `KDGroveRaycaster` to work on refs.
2. Update `RaySceneIntersection` to store ref instead of raw pointer (or both temporarily).
3. Keep public behavior unchanged by adapter methods in `Scene`.

### Phase 6: Dynamic scene migration
1. Refactor `DynObject` matrix extraction/update to index-based access on owning `ScenePart` bulk data.
2. Ensure `DynMovingObject` updates trigger necessary per-part caches (AABB, precomputed edges, etc.).
3. Preserve observer/KD update mechanics.

### Phase 7: Swap-on-repeat and replay
1. Refactor `SwapOnRepeatHandler` to clone/swap whole `ScenePart` polymorphic objects, not primitive vectors.
2. Remove primitive-by-primitive delete/clone code paths.
3. Ensure null/recycle behavior remains identical.

### Phase 8: Serialization
1. Register/serialize new `ScenePart` derived types in scene and KD serialization flows.
2. Remove `AABB` as a registered primitive subtype in polymorphic primitive serialization.
3. Keep `AABB` serialization only where used as utility value/object (`bbox`, `bbox_crs`, `ScenePart::bound`, voxel bounds).
4. Add serialization tests for each new concrete ScenePart.
5. Introduce a serialization format break with explicit version bump and migration note:
   - Previously serialized streams are treated as legacy and unsupported.
   - No reader translation/backward-compatibility layer is required.

### Phase 9: Python/C++ API transition
1. Change Python bindings directly to the new architecture (no deprecation period required).
2. Expose bulk-data-centric APIs for scene-part geometry access and mutation.
3. Remove direct mutable `Primitive*` exposure from Python bindings.
4. Break the inheritance relation in bindings:
   - `AABB` should no longer be exposed as subclass of `Primitive`.
   - Remove `Primitive.is_AABB()`.
   - Keep `AABB` Python class as independent type for bbox utilities.

### Phase 10: Remove legacy Primitive ownership
1. Delete `ScenePart::mPrimitives` ownership model and cleanup manual delete paths.
2. Remove no-longer-needed primitive cloning in dynamic KD update code.
3. Keep primitive classes only if needed as transient adapters; otherwise deprecate.
4. Confirm no test or demo still treats `AABB` as a scene primitive.

## Test Plan (C++ unit focus)

### New/updated C++ tests to add
1. `ScenePartBulkTriangleTest.cpp`
   - bulk storage correctness, centroid, AABB, intersection equivalence.
2. `ScenePartBulkVoxelTest.cpp`
   - voxel bbox, intersections, normal-dependent incidence angles.
3. `ScenePartBulkDetailedVoxelTest.cpp`
   - transmittive/fixed/scaled intersection handling parity.
4. `KDTreeBulkPrimitiveRefTest.cpp`
   - split/build/search parity with legacy behavior.
5. `DynObjectBulkUpdateTest.cpp`
   - matrix read/write on bulk data, post-update ray hits.
6. `SwapOnRepeatBulkTest.cpp`
   - swap lifecycle, keepCRS, null/discard semantics.
7. Extend existing:
   - `src/test/RayIntersectionTest.cpp`
   - `src/test/GroveTest.cpp`
   - `src/test/SerializationTest.cpp` (remove `AABB`-as-primitive assumptions)
   - `src/test/ScenePartSplitTest.cpp` (or replacement if split semantics change)
8. Add dedicated tests for AABB decoupling:
   - `AABBUtilityTest.cpp`: validates AABB intersection/math independent of Primitive hierarchy.
   - Serialization regression test for `Scene::bbox`/`bbox_crs` and `ScenePart::bound` after decoupling.

### Required execution sequence (per project instructions)
1. `ctest` in `build` (C++ tests first).
2. `pytest -vv` in project root.
3. `pre-commit run -a` twice if needed.
4. `pytest --cov ...` with focused scope for refactor modules.

## Expected High-Risk Areas
- KD-tree internals: type migration from `Primitive*` to handle/ref is broad.
- Dynamic motion path: currently depends on mutable `Vertex*` arrays.
- Swap-on-repeat: manual ownership assumptions are deeply tied to primitive pointers.
- Python API compatibility: direct primitive exposure conflicts with bulk-only model.
- Serialization: polymorphic `ScenePart` graph registration and backward compatibility.
- AABB decoupling: legacy serialization/binding/tests currently assume `AABB` can be a `Primitive`.

## Work Breakdown Recommendation
- Use small mergeable PRs in this order:
  1. AABB decoupling + serialization/binding/test fixes required by that decision.
  2. Triangle bulk vertical slice end-to-end, including KD/raycast integration for triangles.
  3. Voxel bulk vertical slice end-to-end.
  4. Detailed voxel bulk vertical slice end-to-end.
  5. Dynamic + swap-on-repeat on top of finalized bulk scene-part API.
  6. Serialization + Python adaptation aligned to final APIs.
  7. Legacy removal and final cleanup.

## Clarifications Needed Before Implementation
None at this time.

## Definition of Done
- No per-primitive heap allocation for scene geometry in normal loading paths.
- Scene geometry ownership is entirely in polymorphic bulk `ScenePart` implementations.
- `AABB` is a non-`Primitive` utility type; no scene/KD primitive container stores `AABB` instances.
- `Scene::primitives` is removed entirely.
- C++ unit tests cover new bulk code paths and pass.
- Existing intersection/dynamic/swap behavior remains functionally equivalent (except explicitly approved API changes).

## Decision Log
- 2026-02-13: `AABB` will no longer inherit from `Primitive`; it will be kept as a standalone utility bounding-volume type. No `AABBScenePart` will be introduced.
- 2026-02-13: Backward compatibility with previously serialized scene data is explicitly not required. A format break is acceptable.
- 2026-02-13: Python bindings do not require deprecation or compatibility bridges; they can be changed directly to match the new bulk-data design.
- 2026-02-13: Migration should use functional checkpoints without large temporary abstractions; prefer end-to-end vertical slices and durable final-design interfaces.
- 2026-02-13: `Scene::primitives` will be removed entirely; no transitional compatibility container/view will be kept.
- 2026-02-13: There are no codified performance goals for this refactor.
- 2026-02-13: Material ownership semantics stay with `std::shared_ptr<Material>`.
