.. currentmodule:: pylabrobot.tecan

pylabrobot.tecan package
========================

Carousel plate hotel
--------------------

.. currentmodule:: pylabrobot.tecan.carousel

The carousel stores plates in cartridges around a rotating plate and hands them over one at a time
at a transfer station. It is sold and documented by Tecan and manufactured by Liconic, whose StoreX
ASCII command set it speaks over a :class:`pylabrobot.io.serial.Serial` transport.

Every operation checks the ready and error flags before writing, waits for the ready bit to return
after a handling command, polls the plate sensors instead of trusting a single instant reading, and
never retries. An interrupted or uncertain operation faults the session until ``stop()`` and
``setup()`` are called again.

Two coordinate systems meet here. The controller addresses *levels* along the lift stroke, while a
cassette has *slots*; how many levels a slot occupies depends on the cassette's shelf pitch. Build
the device with ``cassette_slots`` and ``cassette_level_step``, then use
:meth:`TecanCarousel.level_for_slot` and :meth:`TecanCarousel.slot_for_level` to translate. On the
unit this driver was developed against, a 22-slot standard cassette maps one level per slot and a
7-slot deep-well cassette maps three, so its slots sit at levels 1, 4, 7, 10, 13, 16 and 19.

Handshake, status reads, ``ST 1801`` initialization, rotation through ``WR DM0``, ``ST 1905`` export,
``ST 1904`` import, ``ST 1908``/``ST 1909`` internal moves, ``ST 1800`` soft reset and ``ST 1906``
placement have all been exercised on one LPT220 EVO unit. ``DM200`` is returned undecoded because
this unit's values do not match the documented error table.

Reading a plate barcode (``ST 1712``/``ST 1200``/``ST 1910``/``ST 1710``) is exposed as
:meth:`TecanCarousel.read_barcode`. It is the one operation that changes the positioning mode of the
whole handler, so it sends ``ST 1713``/``RS 1200``/``RS 1910`` afterwards and confirms the exit
through ``DM200`` before returning. The read height is a controller register (``DM27``), not a driver
setting: on the unit the sequence was measured on, the reader aims at the requested level only with
``DM27 = 0``, while the delivered configuration uses 550.

Because the reader's vertical window is narrow and the label does not sit exactly at the nominal
read height, the method sweeps a small band of aim offsets (``sweep``, default 0/50/100/150/200
register units above the level's nominal position) and stops at the first position that returns a
payload; ``BarcodeRead.aim_delta`` reports which one worked. ``DM27`` is read before and restored
after, with a verified readback. Offsets above a level are reached from the level above it, because
the firmware does not honour a negative ``DM27``; level 1 has no level above, so only its downward
offsets exist.

.. autoclass:: TecanCarousel
   :members:

.. autoclass:: CarouselStatus
   :members:

.. autoclass:: BarcodeRead
   :members:

.. autoexception:: CarouselError

.. autoexception:: CarouselRejected
