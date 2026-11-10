"""Accumulate MuJoCo Warp capacity diagnostics at every physics step."""
from __future__ import annotations

import numpy as np
import warp as wp
from mujoco_warp._src.types import OverflowType


@wp.kernel
def _record(flags: wp.array(dtype=int), nefc: wp.array(dtype=int), nacon: wp.array(dtype=int),
            seen: wp.array(dtype=int), peak_efc: wp.array(dtype=int), peak_contacts: wp.array(dtype=int)):
    r = wp.tid()
    seen[r] = seen[r] | flags[r]
    peak_efc[r] = wp.max(peak_efc[r], nefc[r])
    if r == 0:
        peak_contacts[0] = wp.max(peak_contacts[0], nacon[0])


class CapacityAudit:
    def __init__(self, data, worlds: int):
        self.worlds = worlds
        self.seen = wp.zeros(worlds, dtype=int, device=data.overflow.device)
        self.peak_efc = wp.zeros(worlds, dtype=int, device=data.overflow.device)
        self.peak_contacts = wp.zeros(1, dtype=int, device=data.overflow.device)
        self.naconmax, self.njmax = int(data.naconmax), int(data.njmax)

    def record(self, data):
        wp.launch(_record, dim=self.worlds,
                  inputs=[data.overflow, data.nefc, data.nacon, self.seen, self.peak_efc, self.peak_contacts],
                  device=data.overflow.device)

    def result(self):
        flags = self.seen.numpy().astype(np.int64)
        # Solver convergence warnings are recorded separately from storage capacity.
        capacity_mask = int(OverflowType.ALL) & ~int(OverflowType.ITERATIONS | OverflowType.LS_ITERATIONS)
        bad = (flags & capacity_mask) != 0
        peak = int(self.peak_contacts.numpy()[0])
        if peak > self.naconmax:
            bad[:] = True  # shared contact array; assignment to a particular world is unavailable
        return bad, {"method": "overflow bitmask OR and contact/constraint high-water marks at every physics step",
                     "overflow_flags_by_world": flags.tolist(),
                     "overflow_type_values": {x.name: int(x) for x in OverflowType},
                     "capacity_overflow_worlds": int(bad.sum()), "capacity_mask": capacity_mask,
                     "max_active_contacts": peak, "allocated_contacts": self.naconmax,
                     "max_constraints_by_world": self.peak_efc.numpy().tolist(),
                     "allocated_constraints_per_world": self.njmax}
