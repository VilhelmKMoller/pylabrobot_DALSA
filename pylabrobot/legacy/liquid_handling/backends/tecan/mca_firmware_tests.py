import asyncio
import unittest

from pylabrobot.legacy.liquid_handling.backends.tecan.mca_firmware import McaFirmware


class McaFirmwareTests(unittest.TestCase):
  def test_axis_report_sends_selector_and_preserves_response(self):
    calls = []
    async def send(**kwargs):
      calls.append(kwargs)
      return {"data": [100, -148, 2000, 690]}
    reply = asyncio.run(McaFirmware(send).report_axis_parameter(0))
    self.assertEqual(reply, {"data": [100, -148, 2000, 690]})
    self.assertEqual(calls, [{"module": "W1", "command": "RPA", "params": [0]}])

  def test_documented_commands_preserve_parameter_positions(self):
    async def run():
      calls = []
      async def send(**kwargs):
        calls.append(kwargs)
        return "reply"
      mca = McaFirmware(send)
      cases = [
        ("RHP", [1, 48]), ("PIA", []), ("PIB", []), ("PII", [2]),
        ("PAA", [None, -148, 2000]), ("PRA", [10, None, -5]),
        ("BMA", [0, 0, 0, 0]), ("PID", []), ("PPR", [-100]),
        ("PPA", [100]), ("ASH", [1]), ("ATD", []),
        ("ATF", [100, 200, 50, 100, None, 0]),
      ]
      for command, params in cases:
        self.assertEqual(await mca.execute(command, *params), "reply")
      self.assertEqual(calls, [dict(module="W1", command=c, params=p) for c, p in cases])
    asyncio.run(run())

  def test_rejects_invalid_or_ambiguous_before_sending(self):
    async def run():
      calls = []
      async def send(**kwargs):
        calls.append(kwargs)
      mca = McaFirmware(send)
      cases = [("RPA", [9]), ("RPA", [True]), ("RHP", [2, 0]),
               ("RHP", [0, 49]), ("PIA", [16]), ("PIB", [1]),
               ("PII", [4]), ("PII", [0, 10]), ("BMA", [3]),
               ("PAA", [None, None]), ("PPR", [1.2]), ("ATD", [1]),
               ("ATF", [1, 2, 3, 2001]), ("MTR", [1]), ("RPX", [0])]
      for command, params in cases:
        with self.subTest(command=command, params=params):
          with self.assertRaises((ValueError, NotImplementedError)):
            await mca.execute(command, *params)
      self.assertEqual(calls, [])
    asyncio.run(run())
