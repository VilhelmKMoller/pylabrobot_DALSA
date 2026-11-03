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
placement have all been exercised on one LPT220 EVO unit. The barcode positioning mode (``ST 1910``)
is deliberately not exposed, and ``DM200`` is returned undecoded because this unit's values do not
match the documented error table.

.. autoclass:: TecanCarousel
   :members:

.. autoclass:: CarouselStatus
   :members:

.. autoexception:: CarouselError

.. autoexception:: CarouselRejected
