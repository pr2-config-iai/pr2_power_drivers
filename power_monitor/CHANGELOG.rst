^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package power_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (unreleased)
------------------
* Port to ROS 2 (Jazzy): same estimators, parameters (``estimation_method``, ``frequency``,
  ``battery_update_timeout``, ``advanced_log_file``) and topics (``power_state`` latched, i.e.
  transient_local depth 1; ``battery/server2``; ``power_board/state``, subscribed transient_local
  to get the latched state as in ROS 1).
* dynamic_reconfigure replaced by the parameter ``power_state_estimator_`` (0 fuel gauge,
  1 advanced), applied at startup and on change as before.
* Fix a deadlock on ``MASTER_SHUTDOWN``: the update lock was still held when publishing, and
  ``extractObservation()`` took it again.

1.1.10 (2021-05-20)
-------------------
* cmake changes for warnings under noetic
* Contributors: Dave Feil-Seifer

1.1.9 (2021-05-12)
------------------

1.1.8 (2021-03-15)
------------------

1.1.7 (2018-12-17)
------------------

1.1.6 (2018-02-11)
------------------
* change maintainer to ROS orphaned package maintaner
* Contributors: Kei Okada

1.1.5 (2015-01-13)
------------------

1.1.3 (2015-01-12)
------------------
