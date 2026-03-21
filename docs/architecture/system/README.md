# System Architecture State Registry

The system architecture is documented as a set of discrete state models. Each state model
captures a coherent configuration of the system — its elements, interfaces, data flows,
and constraints — at a defined point in the project lifecycle. New state models are added
as the architecture evolves.

Each state model is a folder containing the following documents:

| Document | Contents |
| --- | --- |
| `requirements.md` | Originating requirements that shaped this architecture state |
| `use_cases.md` | Use cases the architecture must satisfy |
| `element_registry.md` | System element registry organized by subsystem or component boundary |
| `dataflow_registry.md` | Data flow types and instance registry |
| `diagrams.md` | System context, data flow, layer architecture, and lifecycle diagrams |
| `icds.md` | Interface control documents |

---

## Defined States

| State | Folder | Status | Description |
| --- | --- | --- | --- |
| Present State | [present/](present/) | Baseline — pending review | Architecture of the currently implemented codebase |
| Future State | [future/](future/) | Initial draft — pending review and authorization | Target architecture per the project roadmap |
