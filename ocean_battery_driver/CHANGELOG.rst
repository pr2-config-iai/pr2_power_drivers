^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ocean_battery_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* Port to ROS 2 (Jazzy): ``ocean_server`` and ``battery_check``, one ament ``CMakeLists.txt``.
* ``ocean_server``: same serial code, diagnostics (``IBPS N``, ``Smart Battery N.M``) and topics
  (``battery/server2``, ``battery/server`` with the ROS 2 field names, ``/diagnostics``); same
  options (``--debug``, ``--count``, ``--dev``) and parameters (``number_of_ports``, ``portN``,
  ``debug_level``, ``lag_timeout``, ``stale_timeout``). An empty ``portN`` counts as unset. The
  port threads share the node and its publishers. log4cxx is no longer used.
* ``battery_check``: same options and output; a ROS clock without a node in place of
  ``ros::Time::init()``.

1.1.10 (2021-05-20)
-------------------
* fixes for boost dependencies
* Contributors: Dave Feil-Seifer

1.1.9 (2021-05-12)
------------------
* added boost_program_options as build dependency
* fixed CMP0048 compiler warning for noetic/focal
* Contributors: Dave Feil-Seifer

1.1.8 (2021-03-15)
------------------

1.1.7 (2018-12-17)
------------------
* Merge pull request `#71 <https://github.com/PR2/pr2_power_drivers/issues/71>`_ from Jntzko/last_battery_update
  Initialize last battery update time with 0
* Initialize last battery update time with 0
  in kinetic initializing a ros time with -1 throws the following error:
  "Time is out of dual 32-bit range"
  Change it to time(0) keeps the previous behaviour, but uses time 0 as initial value.
* Contributors: Yannick Jonetzko

1.1.6 (2018-02-11)
------------------
* change maintainer to ROS orphaned package maintaner
* Contributors: Kei Okada

1.1.5 (2015-01-13)
------------------

1.1.3 (2015-01-12)
------------------
