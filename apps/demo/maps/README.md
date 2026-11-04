# Local demo maps

Place exported FUSMAP2D `.slammap` files here to display them via
`docker compose up --build demo`. The files are read-only in the container.

**New FR079 convention:** use one run directory for all artifacts:

```text
apps/demo/maps/
  fr079-continuous-noloop-001/
    fr079-continuous-noloop-001.slammap
    fr079-continuous-noloop-001.slamreplay.json
    fr079-continuous-noloop-001.report.json
    fr079-continuous-noloop-001.trajectory.json
    fr079-continuous-noloop-001.matches.json
    fr079-continuous-noloop-001.pgm
    fr079-continuous-noloop-001.yaml
    fr079-continuous-noloop-001.review.json
```

The Web UI lists **both** existing flat files and files in one level of
run directories; it refuses symlinked folders/files and does not recurse.
A foldered map is shown as `fr079-continuous-noloop-001 / ...slammap`
and its Recorded LiDAR replay as
`fr079-continuous-noloop-001@fr079-continuous-noloop-001.slamreplay.json`.
Generate new runs with `slam_fr079_offline ... /output/<run-id> --run-directory`;
move existing results only with explicit
`python3 tools/review_artifacts.py organize-fr079 apps/demo/maps/<run-id> --apply`
after previewing. See [experiment artifact convention](../../../docs/experiment-artifacts.md).
They are local data, not source assets or verified world-frame maps.

The map generation registry `CURRENT` and `generation-*.slammap` files may
also be placed here. The UI never mutates, promotes or rolls back maps.

**No setup data required:** When this folder is empty, the standard
Docker demo generates an explicitly labeled **synthetic** workshop
in an isolated temporary directory. Your actual map folder remains
unchanged. A `.slammap` file or `CURRENT` pointer always takes
precedence over the demo fixture; damaged files are reported rather
than hidden behind an example.
