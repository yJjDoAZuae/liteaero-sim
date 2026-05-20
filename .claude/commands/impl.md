# Implementation Plan — `/impl`

Create, update, and maintain structured implementation plans in `docs/implementation/`.

## Usage

```
/impl new <plan-name> [brief scope description]
/impl update <plan-file> [reason for update]
/impl check <plan-file>
/impl check-all
```

- **`new`** — Create a new plan file and register it in the master index.
- **`update`** — Re-analyze a plan after design documentation changes, reorder items by dependency, update states, and raise any inconsistencies.
- **`check`** — Audit one plan for consistency without modifying it; report issues to the user.
- **`check-all`** — Audit every plan registered in the master index.

---

## What an implementation plan is

An implementation plan is a dependency-ordered, state-tracked list of discrete work items needed to produce a software implementation that conforms to the decided design. Plans are not design documents — they refer to design documents. The design documents are authoritative; the plan is a scheduling artifact derived from them.

An implementation plan answers the question: *given that the design decisions are settled, what must be done, in what order, and what is the current state of each item?*

---

## Central location and master index

All plans live in `docs/implementation/`. The file `docs/implementation/PLANS.md` is the master index: every plan is listed there, one line per plan. Cross-plan dependency analysis always starts from the master index.

When creating a new plan with `/impl new`, the skill:

1. Creates `docs/implementation/<plan-name>.md` following the canonical format below.
2. Adds a one-line entry to `docs/implementation/PLANS.md`.

The master index format is:

```markdown
| Plan file | Scope | Status |
| --- | --- | --- |
| [landing_gear_wheel_dynamics.md](landing_gear_wheel_dynamics.md) | Wheel spin ODE: Tustin integrator, bearing drag, substep count | Active |
```

Status values for a plan as a whole: `Draft`, `Active`, `Complete`, `Superseded`.

---

## Work item format

Each work item occupies one row in the plan's item table. The table columns are:

| Column | Description |
| --- | --- |
| `ID` | Unique identifier within the plan: `IP-<PLAN>-<N>` where `<PLAN>` is a short uppercase abbreviation and `<N>` is a sequential integer. Example: `IP-LGW-3`. |
| `Status` | One of: `todo`, `active`, `done`, `blocked`. See status definitions below. |
| `Title` | One-line description of the work. Unambiguous — refers to specific files, classes, or functions where possible. Not a design statement. |
| `Depends on` | Comma-separated list of IDs (within this plan or cross-plan `IP-*` IDs) that must reach `done` before this item can begin. Empty if no dependencies. |
| `Design refs` | Comma-separated list of design document section references: `[filename.md §N](path)`. Every item must cite at least one. |

### Work item status definitions

| Status | Meaning |
| --- | --- |
| `todo` | Not started. All dependencies are satisfied (or this item has none). Ready to begin when instructed. |
| `active` | Implementation is in progress in the current session or a recent session. |
| `done` | Implementation is complete, tests pass, and the change is committed. |
| `blocked` | Cannot begin: either a dependency is not `done`, an open question is unresolved, or a consistency problem exists. A `blocked` item must have a **Blocked reason** noted in its detail row immediately below the table row (see detail format). |

### Example table

```markdown
| ID | Status | Title | Depends on | Design refs |
| --- | --- | --- | --- | --- |
| IP-LGW-1 | done | Add `WheelUnitParams::spindown_time_s` and `spindown_reference_speed_mps` config fields | — | [landing_gear.md §4, OQ-LG-6 resolution](architecture/landing_gear.md) |
| IP-LGW-2 | todo | Compute `c1`, `c2` from spindown params in `WheelUnit::initialize()` | IP-LGW-1 | [landing_gear.md §OQ-LG-6 resolution](architecture/landing_gear.md) |
| IP-LGW-3 | todo | Apply linear+quadratic bearing drag in `WheelUnit::step()` airborne branch | IP-LGW-2 | [landing_gear.md §OQ-LG-6 resolution](architecture/landing_gear.md) |
| IP-LGW-4 | blocked | Replace explicit Euler wheel ODE with Tustin discretization | IP-LGW-3 | [landing_gear.md §4a, OQ-LG-5 resolution](architecture/landing_gear.md) |
```

> **IP-LGW-4 blocked reason:** Depends on IP-LGW-3; also requires substep count to be updated (IP-LGW-6) before the Tustin stability benefit is realized.

---

## Canonical plan file format

```markdown
# <Plan title> — Implementation Plan

**Scope:** One-paragraph description of what this plan covers and which design documents
it derives from. State which open questions are resolved and which (if any) are still open
and therefore blocking items in this plan.

**Design authority:** Comma-separated list of the primary design documents.

**Last updated:** YYYY-MM-DD

---

## Work Items

<work item table>

---

## Notes

Optional free-form section for context that does not fit in the table: ordering rationale,
known risks, external constraints. This section is not design documentation — do not copy
design content here. Use it only to explain scheduling choices.
```

---

## Dependency analysis

When creating or updating a plan:

1. **Read all referenced design documents** in full before writing any work items. Do not infer design decisions from code; read the authoritative document.
2. **Identify atomic work items.** Each item is a single coherent unit of implementation that can be completed and committed independently. Do not bundle unrelated changes. Do not split a single logical change across multiple items unless there is a true dependency boundary.
3. **Map dependencies explicitly.** For each item, ask: *what must be true in the codebase before this item can begin?* If the answer references another work item, record that dependency.
4. **Detect circular dependencies.** If a dependency cycle exists, the plan has a design or decomposition error. Report the cycle to the user and do not produce a plan with cycles.
5. **Order items topologically.** The table rows are ordered so that no item appears before any item it depends on. Items with no mutual dependency may appear in any relative order; prefer the order that minimizes context-switching across subsystems.
6. **Check cross-plan dependencies.** If an item depends on work from a different plan, reference the external ID (`IP-OTHER-N`) explicitly and mark the item `blocked` if the external item is not `done`.

---

## Consistency checks

When running `/impl check` or `/impl check-all`, verify all of the following. Report every failure to the user before making any changes.

| Check | Failure condition |
| --- | --- |
| Design ref exists | A referenced document path or section does not exist on disk. |
| No dangling deps | A `Depends on` ID does not exist in this plan or any registered plan. |
| No cycles | Dependency graph contains a cycle. |
| Topo order | A row appears before a row it depends on. |
| Blocked items annotated | A `blocked` item has no blocked-reason note. |
| OQ alignment | A work item depends on the resolution of an open question that is not yet resolved in the design document. Mark the item `blocked` and reference the OQ ID. |
| Done items verifiable | A `done` item can be verified by reading the referenced source files. If the implementation cannot be found, downgrade to `todo` and report. |
| No unresolved design choices | The plan does not contain work items for features whose design is still described as a proposal or open question in the referenced design documents. |

---

## Open questions during planning

If a dependency relationship is ambiguous, a sequencing decision requires a design choice, or an implementation detail is not specified in the design documents, **do not make a judgment call**. Instead:

1. Document the question using the `/oq` skill in the relevant design document, clearly labeled as an implementation-planning question (distinct from a design-choice question). Use the OQ ID prefix of the relevant document (e.g., `OQ-LG-N`).
2. Mark the affected work items `blocked` with a reference to the new OQ.
3. Continue writing all items that do not depend on the unresolved question.

Implementation planning open questions concern sequencing and decomposition. Design open questions concern what the software should do. Keep them separate.

---

## Updating a plan after design changes

When the user updates a design document (e.g., closes an open question or changes the architecture), run `/impl update <plan-file>` to:

1. Re-read the updated design documents.
2. Add new work items for newly decided design elements.
3. Remove or mark `superseded` any work items that no longer apply.
4. Update all `blocked` items whose blocking condition has been resolved.
5. Re-check topological order and re-sort if needed.
6. Update the `Last updated` date.

A documentation update that closes an open question is not an instruction to begin implementation. Updating the plan is documentation work, not implementation work.

---

## Style rules

- Work item titles use imperative mood and name specific artifacts: "Add `bearing_drag_nsm_per_rad` to `WheelUnitParams`" not "Update the wheel parameters."
- Design refs use Markdown relative links with section anchors where possible.
- Do not reproduce design content in the Notes section. Cross-reference it.
- Do not add work items for tasks that are already implied by the project's standing TDD rule (e.g., "write tests" is not a separate item — it is part of every implementation item). Exception: if a test requires substantial standalone work (e.g., a new test fixture or scenario harness), it may be its own item with an explicit dependency on the item it tests.
- `schema_version` changes and proto field additions are always separate work items from the C++ implementation changes that use them, because they may need to be done in a specific order to avoid breaking existing serialization tests.
