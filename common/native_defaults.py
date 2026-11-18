"""Remove simulator tuning from an MJCF while retaining the physical task.

This is deliberately limited to numerical contact/solver settings. Mass,
geometry, friction, gravity, timestep, explicit contact pairs, and contact
activation geometry remain the task definition. The resulting model must be
audited after compilation before calling it an out-of-box baseline.
"""

from xml.etree import ElementTree as ET


OPTION_TUNING = {
    "solver", "integrator", "cone", "jacobian", "impratio", "iterations",
    "tolerance", "noslip_iterations", "noslip_tolerance", "ls_iterations",
    "ls_tolerance", "mpr_iterations", "mpr_tolerance", "sdf_iterations",
    "sdf_initpoints",
}
CONTACT_TUNING = {"solref", "solimp", "solreffriction"}


def native_default_mjcf(xml: str) -> str:
    """Use MuJoCo's compiled defaults for contact response and solver knobs."""
    root = ET.fromstring(xml)
    for option in root.iter("option"):
        for name in OPTION_TUNING:
            option.attrib.pop(name, None)
    for element in root.iter():
        for name in CONTACT_TUNING:
            element.attrib.pop(name, None)
    return ET.tostring(root, encoding="unicode")
