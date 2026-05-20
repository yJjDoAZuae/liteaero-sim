---
description: Add or close a fully documented open question in a design document. Usage: /oq <document-path> [brief description]
---

# Open Question — `/oq`

Add a fully documented open question to a design document.

## Usage

```
/oq <document-path> [brief description of the question]
```

## What this skill does

1. **Reads the target document** to understand its current open questions (to assign the next ID) and the subsystem context (to write a self-contained description).
2. **Drafts the open question** following the canonical structure below.
3. **Inserts the new entry** — both a one-line row in the Open Questions summary table and a full subsection in the Open Questions section.
4. **Does not implement anything.** The skill only adds documentation. Implementation begins only when the user explicitly instructs it.

## Canonical structure for each open question

Every open question must contain all five of the following parts in order:

### 1. Summary table row

One row added to the summary table at the top of the Open Questions section:

```markdown
| OQ-<ID>-<N> | <one-line summary> | <Blocking / Not blocking [qualifier]> |
```

### 2. Full subsection heading

```markdown
### OQ-<ID>-<N> — <Title>
```

### 3. Problem description

A self-contained description of the problem. **Must not require reading the rest of the document to understand.** Write it as if the reader has general engineering knowledge but has not read this document:

- Define every term and symbol used.
- State what the current implementation does and why that is insufficient or uncertain.
- Quantify the impact where possible (error magnitude, frequency, affected scenarios).
- Identify which use cases are affected and which are not.

### 4. Alternatives

An enumerated list. **Every viable alternative must be included** — do not omit options because they seem unlikely. For each alternative:

- Give it a short name and number (1, 2, 3 …).
- Describe what it involves.
- State its benefits.
- State its drawbacks.
- State any prerequisite work or information needed to pursue it.

### 5. Recommendation

State a specific recommended alternative and the reasoning. The recommendation is a design opinion, not a decision — the decision is made by the user when they close the question. If the answer genuinely depends on information that does not yet exist (e.g., flight test data, a performance measurement), say so explicitly and describe what information is needed before a recommendation can be made.

## Closing an open question

When the user provides a design choice and instructs a documentation update to close the open question:

1. Remove the row from the summary table.
2. Replace the full subsection with a concise **Resolution note** that records:
   - The chosen alternative.
   - The rationale given by the user (or inferred from context if clear).
   - Any constraints or caveats attached to the choice.
3. Update any other sections of the document that referenced the open question to reflect the resolved design.
4. **Do not begin implementation.** A documentation update closing an open question is not an instruction to implement.

## Style rules

- The problem description must stand alone — no "as described in §3b" or "as noted above" references for the core problem statement. Cross-references to other sections are permitted for supporting detail only.
- Use SI units and the project naming conventions (`snake_case` for identifiers, American English spellings).
- Quantify impacts with numbers where possible. Vague statements like "may cause issues" are not acceptable.
- Do not advocate for a specific alternative in the Alternatives section — save advocacy for the Recommendation.
- If fewer than two viable alternatives exist, reconsider whether the question is genuinely open or whether the answer is already determined.
