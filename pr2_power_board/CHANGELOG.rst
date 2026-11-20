^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_power_board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* Port to ROS 2 (Jazzy): the service definitions and the power board driver ``power_node2``.
  ``power_node`` (the old protocol), the simulator and the scripts are not ported.
* ``power_node2``: same protocol code; same interfaces as ROS 1 (``~/control``, ``~/control2``,
  ``~/state`` latched, ``/diagnostics``, ``battery/server2``). The board address can be given as
  ``--address=...`` as before or as the ``address`` parameter; ``sample_frequency`` and
  ``transition_frequency`` are parameters.
* Fix: ``boost::mutex::scoped_lock(library_lock_);`` declared an unused lock named
  ``library_lock_`` instead of locking the mutex, so the receive and publish threads were never
  synchronized. They now use ``std::lock_guard``.

1.1.10 (2021-05-20)
-------------------
* fixes for boost dependencies
* cmake changes for warnings under noetic
* Contributors: Dave Feil-Seifer

1.1.9 (2021-05-12)
------------------
* added dependency to pr2_power_drivers for boost_program_options
* Contributors: Dave Feil-Seifer

1.1.8 (2021-03-15)
------------------
* fix for python3 (`#73 <https://github.com/pr2/pr2_power_drivers/issues/73>`_)
  * fix by 2to3 -w -f except .
  * fix by 2to3 -w -f print .
* Contributors: Kei Okada

1.1.7 (2018-12-17)
------------------

1.1.6 (2018-02-11)
------------------
* change maintainer to ROS orphaned package maintaner
* pr2_power_board: add missing include dirs
* Contributors: Furushchev, Kei Okada

1.1.5 (2015-01-13)
------------------

1.1.3 (2015-01-12)
------------------
