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
after a handling command, compares the plate sensors with the expected outcome, and never retries.
An interrupted or uncertain operation faults the session until ``stop()`` and ``setup()`` are called
again.

Status reads, export (``ST 1905``), soft reset (``ST 1800``) and placement (``ST 1906``) have been
observed on one LPT220 EVO unit. The remaining handling commands, and the plate-ready flag, follow
the command reference but have not been exercised on hardware. The unit's ``DM200`` values do not
match the documented error table, so ``process_status`` is returned undecoded.

.. autoclass:: TecanCarousel
   :members:

.. autoclass:: CarouselStatus
   :members:

.. autoexception:: CarouselError

.. autoexception:: CarouselRejected
