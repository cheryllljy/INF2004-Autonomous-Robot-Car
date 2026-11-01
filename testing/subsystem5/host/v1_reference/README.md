# v1 reference (read-only)

Unmodified copies of the pre-CR1 Buddy 5 geometry and headers
(`obstacle_geometry_v1.c` is `app_program/old_src/obstacle_geometry.c`).

They are compiled ONLY into the host test `test_equiv`, which feeds the
same 20 000 random sweeps to the v1 geometry (unpacked samples) and the
v2 geometry (packed samples) and requires every profile field and the
action to match. That is the CR1 "no loss of required functionality"
evidence for the geometry. Do not edit these files.
