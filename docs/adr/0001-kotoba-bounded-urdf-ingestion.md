# ADR 0001: Kotoba bounded URDF ingestion slice

Status: qualified

`src/urdf_query.kotoba` owns the pure XML ingestion slice that can already be
expressed with the sealed Kotoba XML and decimal ABIs. It queries robot name,
ordered link names, typed finite mass values, ordered joint name, kind, parent,
and child attributes, plus typed finite lower, upper, effort, and velocity
limits. It does not replace the full CLJC `ArticulatedSystem` constructor:
optional pose/inertia vector fields, graph validation, and structured
record construction remain explicit later steps.

The query module is authority-free and has no DOM, filesystem, network, entity,
DTD, selector, or callback access. It inherits the compiler's 65,536-byte,
2,048-node, depth-32, 32-attribute, and 32-path-segment limits. Missing string
attributes are typed `[:option :string]`; missing, malformed, or non-finite
numeric attributes are typed `[:option :f64]` none. Malformed or excessive XML
traps.

Qualification compares both committed real fixtures, `cartpole.urdf` and
`hizukue.urdf`, against the existing CLJC `parse-urdf` oracle. The same
`.kotoba` source and input are executed by the Kotoba reference evaluator,
restricted JavaScript, and actual typed Wasm through the pinned compiler's
browser host. Robot/link/joint identity, joint topology, and mass attribute
vectors agree exactly.

This establishes real URDF ingestion without claiming that the full CLJC
parser has been migrated. The next language gap is pose/inertia vector parsing
plus typed record/graph assembly; the physics solver remains a separate
Kami engine provider boundary.

Evidence: kotoba-lang/compiler PR #137 merged as
`48efc8aec36f7136cd0640b67a439d4f06d5d88c`; this repository pins that exact
commit. `clojure -M:test` passes 9 tests and 42 assertions, including both real
fixtures across the CLJC oracle, Kotoba reference evaluator, restricted
JavaScript, and actual typed Wasm browser host.
