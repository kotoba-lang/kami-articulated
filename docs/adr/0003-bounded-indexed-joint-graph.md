# ADR 0003: Bounded indexed joint graph

Status: implemented and qualified

`joint-graph-valid` now delegates to an order-independent indexed validator.
The admitted profile contains 1 through 20 uniquely named links and exactly
`links - 1` joints. Link and joint declaration order has no semantic effect.
Twenty is a derived safety bound: the sealed typed map permits 31 entries, but
the complete map plus two union-find i64 vectors must remain below the
64-node compound-value budget and the worst admitted validation path must stay
inside the non-replenishable 512-call fuel budget.

The validator builds a persistent `[:map :string :i64]` from link identity to
index. Duplicate or missing link names fail closed. Each joint then resolves
its parent and child through that sealed index, rejects missing identities,
duplicate joint names, and a child already owned by an earlier joint, and
unions the two indices using persistent parent and rank vectors. Finding both
endpoints in the same union-find component rejects self-edges and cycles.

With `n - 1` admitted edges, no cycle, and every endpoint resolved to one of
the `n` links, the undirected graph is connected. Unique child ownership gives
every non-root link at most one directed parent. Thus multiple parents,
cycles, disconnected components, unknown references, and duplicate identities
are rejected without depending on source order or performing an unbounded
ancestor traversal.

Qualification retains the real cartpole and hizukue CLJC-oracle matrix and the
existing unsafe graph matrix. It additionally admits a deliberately scrambled
four-link tree and a maximum-size twenty-link chain whose link and joint lists
are both reversed. The same `.kotoba` module passes through the reference
evaluator, restricted JavaScript, and actual browser-hosted typed Wasm without
increasing any compiler/runtime limit.

Evidence: `clojure -M:test` passes 12 tests / 63 assertions with zero failures
or errors.
