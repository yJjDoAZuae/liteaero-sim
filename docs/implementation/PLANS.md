# Implementation Plans — Master Index

All structured implementation plans are listed here. Every plan must be registered in this
index before any of its work items are begun. Cross-plan dependency analysis starts from
this file.

Plans are created and maintained using the [`/impl` skill](../../.claude/commands/impl.md).

---

| Plan file | Scope | Status |
| --- | --- | --- |
| [landing_gear_dynamics.md](landing_gear_dynamics.md) | Wheel kappa fix, rolling-condition clamp, n_contact_z_filt filter, Tustin ODE (OQ-LG-5), airborne bearing drag (OQ-LG-6) | Active |

---

## Notes

The files [`aircraft_serialization.md`](aircraft_serialization.md) and
[`equations_of_motion.md`](equations_of_motion.md) predate the `/impl` skill and are
free-form design notes, not structured implementation plans. They are not registered above
and do not conform to the canonical plan format. They should be migrated or superseded
when the subsystems they cover are next worked on.
