Not applicable in this directory's own right: this is the ECAM/FWS bridge
(turns other areas' registered `EcamAlert`s into FlyByWire's own tables —
see `docs/deep/ecam_bridge.md`), not a physical system model. It registers
no `ComponentDef`/`FailureDef` of its own, so there is nothing to list in
the `ATA | proposed name | model element | magnitude | effect` format the
other areas use.

Every physical failure that reaches the cockpit through this bridge is
listed in the registering area's own `FAILURES.md` (`EcamAlert.raised_by`
names the failure ids); this bridge only carries their alert's title,
procedure and trigger condition into FlyByWire's display.
