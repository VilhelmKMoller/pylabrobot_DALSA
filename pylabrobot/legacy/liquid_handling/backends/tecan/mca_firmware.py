"""MCA command parameters from Tecan 394562 V1.0 pp. 18, 20, 32, 34–39.

This adapter only validates protocol shape and documented fixed bounds. It does
not validate installed travel limits, geometry, state, completion, or safety.
It is not connected to EVO_backend; supplying a real sender can move hardware.
PIA/PIB selectors, PII overrides and MTR are withheld due to manual ambiguity.
"""


class McaFirmware:
  """Pass commands to an injected async sender using EVO's keyword convention."""

  def __init__(self, sender, module="W1"):
    self._sender = sender
    self.module = module

  async def report_axis_parameter(self, selector):
    return await self.execute("RPA", selector)

  async def execute(self, command, *params):
    """Validate and send a supported firmware command; return sender response unchanged."""
    # Bounds of None mean installation-specific; they are not safety checks.
    specs = {
      "RPA": (1, 1), "RHP": (2, 2), "PIA": (0, 0), "PIB": (0, 0),
      "PII": (1, 1), "PAA": (1, 4), "PRA": (1, 4), "BMA": (1, 4),
      "PID": (0, 0), "PPR": (1, 1), "PPA": (1, 1), "ASH": (1, 1),
      "ATD": (0, 0), "ATF": (4, 6),
    }
    if command not in specs:
      raise NotImplementedError(f"Unsupported or ambiguous MCA command: {command}")
    lo, hi = specs[command]
    if not lo <= len(params) <= hi:
      raise ValueError(f"{command} expects {lo}..{hi} parameters")
    optional = {"PAA", "PRA", "BMA"}
    for i, value in enumerate(params):
      may_omit = command in optional or (command == "ATF" and i >= 4)
      if value is None and may_omit:
        continue
      if type(value) is not int:
        raise ValueError(f"{command} parameter {i} must be an integer")
    if command in optional and all(value is None for value in params):
      raise ValueError(f"{command} must select at least one axis")
    bounds = {
      "RHP": {0: (0, 1), 1: (0, 48)}, "PII": {0: (0, 3)},
      "BMA": {i: (0, 2) for i in range(4)}, "ASH": {0: (0, 1)},
      "ATF": {3: (0, 2000), 4: (1, 249), 5: (0, 1)},
    }
    if command == "RPA" and params[0] not in (*range(9), 10, 11, 12):
      raise ValueError("Invalid RPA selector")
    for i, (minimum, maximum) in bounds.get(command, {}).items():
      if i < len(params) and params[i] is not None and not minimum <= params[i] <= maximum:
        raise ValueError(f"{command} parameter {i} outside documented bounds")
    return await self._sender(module=self.module, command=command, params=list(params))
