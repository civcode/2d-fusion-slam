# Local demo maps

Place exported FUSMAP2D `.slammap` files here to display them via
`docker compose up --build demo`. The files are read-only in the container.
They are local data, not source assets or verified world-frame maps.

The map generation registry `CURRENT` and `generation-*.slammap` files may
also be placed here. The UI never mutates, promotes or rolls back maps.

**No setup data required:** When this folder is empty, the standard
Docker demo generates an explicitly labeled **synthetic** workshop
in an isolated temporary directory. Your actual map folder remains
unchanged. A `.slammap` file or `CURRENT` pointer always takes
precedence over the demo fixture; damaged files are reported rather
than hidden behind an example.
