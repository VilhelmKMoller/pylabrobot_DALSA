"""Execute only the notebook's isolated in-memory X wrapper cells."""
import asyncio
import json
from pathlib import Path


def test_notebook_simulated_wrapper_coordinates_all_three_arm_names():
  root = Path(__file__).resolve().parents[5]
  notebook = json.loads((root / 'docs/user_guide/00_liquid-handling/tecan/prep_basic_EVO.ipynb').read_text())
  cells = {''.join(cell['source']).splitlines()[0]: ''.join(cell['source'])
           for cell in notebook['cells'] if cell['cell_type'] == 'code' and cell['source']}
  namespace = {}
  exec(cells['# Offline X-wrapper setup (no backend or hardware)'], namespace)

  async def exercise():
    wrapper = namespace['new_simulated_x_wrapper']()
    moves = await wrapper.move_x('mca', 9100)
    assert [(move.arm, move.target) for move in moves] == [('roma', 11540), ('mca', 9100)]
    for name, target in [('liha', 100), ('roma', 12000)]:
      wrapper = namespace['new_simulated_x_wrapper']()
      assert (await wrapper.move_x(name, target))[-1].arm == name
  asyncio.run(exercise())
